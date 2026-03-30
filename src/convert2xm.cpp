/*
 * TFMX to XM (Extended Module) converter
 * Author: Domy (domy@noisebay.org)
 *
 * Converts Amiga TFMX modules (mdat + smpl file pair) into FastTracker II
 * compatible XM files for playback in modern trackers (e.g. Schism Tracker).
 *
 * TFMX (The Final Musicsystem eXtended) by Chris Huelsbeck uses an event-based
 * architecture where "macros" define instrument behaviour (sample playback,
 * envelopes, vibrato) and "patterns" contain note/timing events processed on a
 * per-tick basis. XM, in contrast, is row-based: each row fires once, then the
 * tracker advances after N ticks (the "speed" value).
 *
 * Conversion strategy:
 *   1. Load the entire mdat into a flat buffer for random access (required
 *      because TFMX pattern commands like pGsPt/pJump can reference arbitrary
 *      file offsets).
 *   2. Simulate TFMX playback tick-by-tick, writing note/volume events into a
 *      linear XM row grid ("bigGrid").
 *   3. Post-process: apply effective volumes (from macro mAddVol/mSetVol),
 *      vibrato (XM effect 4xy), sustain/release envelope fades (XM volume
 *      slide 6x/7x), and per-note stereo panning (XM effect 8xx).
 *   4. Slice the grid into 64-row XM patterns and write instruments with
 *      delta-encoded 8-bit sample data.
 *
 * XM layout reference: "The Unofficial XM File Format Specification" by
 * Kamenar & CelerSMS. Key constants:
 *   - XM header size = 276 bytes (at file offset 60)
 *   - Instrument header size = 263 bytes (with one sample per instrument)
 *   - Sample header = 40 bytes appended to instrument block
 *   - Pattern header = 9 bytes (size 4 + packing 1 + rows 2 + datasize 2)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#ifndef _WIN32
#include <arpa/inet.h>
#endif

#include "tfmx.h"
#include "convert2xm.h"

#ifndef TFMXPLAY_VERSION
#define TFMXPLAY_VERSION "0.0.7"
#endif

/* Win32 lacks arpa/inet.h — provide byte-swap helpers for big-endian TFMX data */
#ifdef _WIN32
static unsigned short ntohs_win(unsigned short x) {
  return (unsigned short)((x>>8)|(x<<8));
}
static unsigned int ntohl_win(unsigned int x) {
  return ((x&0xff)<<24)|((x&0xff00)<<8)|((x&0xff0000)>>8)|((x&0xff000000)>>24);
}
#define ntohs ntohs_win
#define ntohl ntohl_win
#endif

/*
 * Map TFMX volume (0-64) to XM volume (0-64) with a non-linear gain boost
 * that compensates for the channel-count difference (TFMX 4 ch vs XM 8+).
 *
 * Below v=30 the curve is a straight 3/2 ramp — this lifts quieter
 * instruments (bass, pads) so they stay audible in an 8-channel mix.
 * Above v=30 a sqrt-shaped transition smoothly approaches 64 at v=64.
 */
static int scaleVol(int v) {
  if (v <= 0) return 0;
  if (v >= 64) return 64;
  if (v <= 30) return (v * 3 + 1) / 2;
  double t = (v - 30) / 34.0;
  int r = 45 + (int)(19.0 * sqrt(t) + 0.5);
  if (r > 64) r = 64;
  return r;
}

/* XM uses little-endian byte order throughout */
static void writeU16(FILE* f, unsigned short v) {
  unsigned char b[2] = { (unsigned char)(v & 0xff), (unsigned char)(v >> 8) };
  fwrite(b, 1, 2, f);
}
static void writeU32(FILE* f, unsigned int v) {
  unsigned char b[4] = {
    (unsigned char)(v & 0xff),
    (unsigned char)((v>>8) & 0xff),
    (unsigned char)((v>>16) & 0xff),
    (unsigned char)(v >> 24)
  };
  fwrite(b, 1, 4, f);
}

/*
 * Song-specific patches.  Some TFMX songs produce conversion artifacts
 * (hanging notes, volume escalation) that can't be fixed generically
 * without breaking other songs.  Patches are matched by the mdat
 * filename's basename (e.g. "mdat.T2_Title") and subsong number.
 */
/* XM pattern cell — matches the five fields of an XM packed note event. */
struct XMCell { unsigned char note, inst, vol, fx, param; };

/* One targeted fix within a song patch.  Each fix addresses a specific
 * conversion artefact on one channel/instrument in a given row range.
 * Fields set to -1 (or false) are inactive / wildcards. */
struct SongFixEntry {
  int channel;      /* XM channel, 0-based (-1 = any) */
  int instrument;   /* XM instrument, 1-based (-1 = any) */
  int fromRow;      /* first row to apply fix (-1 = start of song) */
  int toRow;        /* last row (exclusive) to apply fix (-1 = end of song) */
  int orphanGap;    /* insert note-off after this many silent rows (-1 = default) */
  int maxVolume;    /* cap volume to this value; 0 + fromRow/toRow = mute range (-1 = off) */
  int minNoteGap;   /* min rows between note-ons; faster retriggers blanked (-1 = off) */
  int volSlideDown;  /* XM vol-slide-down per tick on empty rows (1-15, -1 = off) */
  bool suppress;    /* blank all matching note-ons in range */
};

/* Song-specific patch, identified by up to three methods (tried in order):
 * 1. mdat filename basename  2. mdat content hash  3. note fingerprint.
 * Contains up to 8 fix entries (terminated by a sentinel with ch=-1, inst=-1). */
struct SongPatch {
  const char* mdatName;         /* e.g. "mdat.T2_Title" — primary match key */
  unsigned int mdatHash;        /* djb2 hash of mdat file content */
  unsigned int noteFingerprint; /* djb2 hash of first 200 note events in bigGrid */
  int subsong;                  /* -1 = all subsongs */
  const char* label;            /* human-readable name for log output */
  SongFixEntry fixes[8];
};

/* djb2 hash of raw mdat file bytes — used as fallback song identifier. */
static unsigned int hashMdat(const unsigned char* buf, size_t len) {
  unsigned int h = 5381;
  for (size_t i = 0; i < len; i++)
    h = ((h << 5) + h) ^ buf[i];
  return h;
}

/* Build a fingerprint from the first N note-on events in the converted
 * bigGrid.  Hashes (channel, note, instrument) triples so the same musical
 * content produces the same fingerprint even if the file was re-dumped. */
#define NOTE_FP_EVENTS 200
static unsigned int computeNoteFingerprint(const XMCell* grid, int totalRows, int numChans) {
  unsigned int h = 5381;
  int count = 0;
  for (int row = 0; row < totalRows && count < NOTE_FP_EVENTS; row++) {
    for (int c = 0; c < numChans && count < NOTE_FP_EVENTS; c++) {
      const XMCell* cell = &grid[row * numChans + c];
      if (cell->note > 0 && cell->note < 97 && cell->inst > 0) {
        h = ((h << 5) + h) ^ (unsigned int)c;
        h = ((h << 5) + h) ^ (unsigned int)cell->note;
        h = ((h << 5) + h) ^ (unsigned int)cell->inst;
        count++;
      }
    }
  }
  return h;
}

/* ---- Song patch table ----
 * Each entry targets a specific mdat + subsong and lists up to 8 fixes.
 * Fix rows terminated by sentinel {-1,-1,...}.
 *                                                                                ch  inst  from   to   oGap  maxV  mnGap vSlD  suppr */
static const SongPatch songPatches[] = {
  { "mdat.T2_Title",  0x5adadcd4u, 0xd9df46bbu, 0, "T2_Title", {
    {  1,  -1, 1665, 1666,   -1,    0,   -1,  -1, false }, /* Pattern-Fix: kill Inst14 ring at pat26 row1 */
    {  0,   5,   -1,  -1,   -1,   42,   -1,  -1, false }, /* Ch00 Inst05: volume cap 42 */
    { -1,  -1,   -1,  -1,   -1,   -1,   -1,  -1, false }, /* sentinel */
  }},
  { "mdat.T2_World1", 0x87b88ce6u, 0x5493b2deu, 3, "T2_World1 Sub03", {
    {  2,  64,   -1,  -1,   48,   -1,   -1,  -1, false }, /* Ch02 Inst64: orphan gap 48 only */
    { -1,  -1,   -1,  -1,   -1,   -1,   -1,  -1, false },
  }},
  { "mdat.T2_World1", 0x87b88ce6u, 0xbe3a7040u, 4, "T2_World1 Sub04", {
    {  1,  54,   -1,  -1,   12,   -1,   -1,  -1, false }, /* Ch01 Inst54 bass: note-off after 12 rows silence */
    { -1,  -1,   -1,  -1,   -1,   -1,   -1,  -1, false },
  }},
  { "mdat.title",     0xb2eae944u, 0xcde8a36fu, 0, "T1_Title", {
    { -1,  29,   -1,  -1,   -1,   42,   -1,  -1, false }, /* Drums (Inst29/Macro28): cap volume */
    { -1,  24,   -1,  -1,   -1,   35,   -1,  -1, false }, /* Transient (Inst24/Macro23): cap volume */
    { -1,  -1,   -1,  -1,   -1,   -1,   -1,  -1, false },
  }},
  { "mdat.T2_World2", 0x12a3f169u, 0xdaf49cf2u, 0, "T2_World2 Sub00", {
    {  0,  -1,    0,   1,   -1,    0,   -1,  -1, false }, /* Ch00 row0: vol=0 to kill loop bleed */
    { -1,  -1,   -1,  -1,   -1,   -1,   -1,  -1, false },
  }},
  { NULL, 0, 0, -1, NULL, {{ -1, -1, -1, -1, -1, -1, -1, -1, false }} },
};

/* Extract the filename portion from a full path (after last / or \). */
static const char* basenameOf(const char* path) {
  const char* s = strrchr(path, '/');
  if (!s) s = strrchr(path, '\\');
  return s ? s + 1 : path;
}

/* Three-tier song identification: try name first (fast, human-readable),
 * then content hash (works for renamed files), then note fingerprint
 * (works even if the binary was patched but the music is the same). */
static const SongPatch* findSongPatch(const char* mdatPath,
                                       unsigned int hash,
                                       unsigned int noteFP,
                                       int subsong,
                                       const char** matchMethod) {
  const char* base = basenameOf(mdatPath);

  for (int pass = 0; pass < 3; pass++) {
    for (int i = 0; songPatches[i].label; i++) {
      if (songPatches[i].subsong != -1 && songPatches[i].subsong != subsong) continue;
      bool match = false;
      if (pass == 0 && songPatches[i].mdatName)
        match = (strcmp(base, songPatches[i].mdatName) == 0);
      else if (pass == 1 && songPatches[i].mdatHash)
        match = (songPatches[i].mdatHash == hash);
      else if (pass == 2 && songPatches[i].noteFingerprint)
        match = (songPatches[i].noteFingerprint == noteFP);
      if (match) {
        static const char* methods[] = { "name", "hash", "notes" };
        *matchMethod = methods[pass];
        return &songPatches[i];
      }
    }
  }
  *matchMethod = NULL;
  return NULL;
}

/*
 * Sample metadata extracted from a TFMX macro definition.
 * TFMX macros can define multi-phase instruments (e.g., a drum click attack
 * followed by a bass sustain) by issuing mSetBegin/mSetLen before AND after
 * mOn. We track both phases and prefer the post-mOn (sustain) sample.
 */
struct MacroSampleInfo {
  int start;       /* byte offset into the smpl file */
  int len;         /* sample length in bytes (TFMX stores words; we convert) */
  int loopStart;   /* loop start offset in bytes, relative to sample start */
  int loopLen;     /* loop length in bytes; 0 = no loop */
  bool oneShot;    /* true if macro contains mOneShot (disables looping) */
  bool hasSetLoop; /* true if macro contains mSetLoop (explicit loop point) */
  int addNote;     /* pitch transposition in semitones from mAddNote */
  int atkStart;    /* attack phase sample offset in smpl (-1 = none) */
  int atkLen;      /* attack phase sample length in bytes */
  int setNoteVal;  /* mSetNote value before mOn; -1 = no fixed attack pitch */
  int sweepAmt;    /* mAddBegin: bytes per frame to shift loop start (0=none) */
  int sweepFrames; /* frames of sweep before mOneShot/mStop */
  int sweepHalfPeriod; /* data[0] from mAddBegin: frames per ping-pong half-cycle */
  int sweepAmt2;   /* second mAddBegin amount (0=none); enables forward-backward trajectory */
  int sweepReverseFrame; /* frame index where sweep direction changes to sweepAmt2 */
  bool negSweep;   /* true if macro has mAddBegin with negative amount (unbaked sweep) */
  int negSweepAmt; /* negative sweep bytes per frame (negative value) */
  int negSweepHp;  /* halfperiod for negative sweep ping-pong */
};

/*
 * Effect parameters extracted from a TFMX macro, pre-converted to XM scale.
 *
 * TFMX volume model:
 *   - Base volume = patVol (0-15) from pattern data
 *   - If mAddVol present:  effective = mAddVol + patVol * 3  (mAddVol is SIGNED)
 *   - If mSetVol present:  effective = mSetVol  (absolute override)
 *   - Fallback:            effective = patVol * 3
 *   This maps the 0-15 pattern range onto roughly 0-48, fitting XM's 0-64.
 *
 * TFMX envelopes have two phases, separated by mWaitUp (wait for pKeyUp):
 *   1. Sustain phase: volume slide while note is held (e.g., slow fade)
 *   2. Release phase: volume slide after key-up (typically fast fade to 0)
 *   Both are converted to XM volume column slide commands (6x down / 7x up).
 */
struct MacroEffectInfo {
  int addVol;         /* mAddVol data[2] as signed char; -999 = not present */
  int setVol;         /* mSetVol data[2] absolute volume; -1 = not present */
  int vibSpeed;       /* XM vibrato speed (effect 4, high nibble); 0 = off */
  int vibDepth;       /* XM vibrato depth (effect 4, low nibble) */
  bool vibAfterRelease; /* true if vibrato only appears after mWaitUp (release vibrato) */
  int sustainSlide;   /* volume slide per row during sustain; negative = down */
  int sustainDelay;   /* rows to wait before sustain slide begins (from mWait) */
  int sustainTarget;  /* TFMX envelope target vol (0-64); slide stops here */
  int releaseSlide;   /* volume slide per row after pKeyUp; negative = down */
  int releaseTarget;  /* volume where first release stage stops (0 = fade to 0) */
  int releaseSlide2;  /* second release stage: slide from releaseTarget to 0 */
  bool autoRelease;   /* true if release should auto-start when sustain target reached (two-stage sustain before mWaitUp) */
  int stutterOnFrames;  /* frames of sound per stutter cycle; 0 = no stutter */
  int stutterOffFrames; /* frames of silence per stutter cycle */
  int stutterVol;       /* volume during the "on" phase of stutter (0-64) */
  int stutterDelay;     /* macro frames before stutter loop starts */
  int stutterFadeCycles;/* cycles until stutter silences (from mAddBegin) */
  int retriggerFrames;  /* frames per DMA retrigger cycle (mOff/mOn in loop); 0 = none */
};

/*
 * Extract sample parameters from a TFMX macro definition.
 *
 * Walks through macro opcodes to find mSetBegin (sample offset), mSetLen
 * (sample length in words), mOn (trigger playback), mSetLoop, mOneShot,
 * and mAddNote. For multi-phase macros that redefine the sample after mOn
 * (e.g., a short percussive click followed by a looping bass tone), the
 * post-mOn parameters take priority — they represent the sustained sound
 * that the listener perceives as the instrument's timbre.
 */
static MacroSampleInfo getMacroSample(const TFMXMacroData macro[256]) {
  MacroSampleInfo info = { 0, 512, 0, 0, false, false, 0, -1, 0, -1, 0, 0, 0, 0, 0, false, 0, 0 };
  int curStart = 0;
  int curLenWords = 256;   /* TFMX lengths are in 16-bit words */
  bool gotAny = false;
  int addNotePreOn = 0;    /* mAddNote before mOn (used if no post-mOn) */
  int addNotePostOn = -999; /* mAddNote after mOn — preferred for sustain */
  bool afterOn = false;
  int preOnSetNote = -1;   /* mSetNote value found before mOn */
  bool gotAddBegin = false;
  bool afterWaitUp = false;

  int onStart = 0, onLen = 512;
  int firstOnStart = 0, firstOnLen = 512;
  bool hadFirstOn = false;
  int postStart = -1, postLen = -1;

  for (int i = 0; i < 256; i++) {
    switch (macro[i].op) {
      case mSetBegin:
        curStart = (macro[i].data[0]<<16)|(macro[i].data[1]<<8)|(macro[i].data[2]);
        if (afterOn && postStart < 0) postStart = curStart;
        break;
      case mSetLen:
        curLenWords = (macro[i].data[1]<<8)|(macro[i].data[2]);
        if (curLenWords <= 0) curLenWords = 256;
        if (afterOn && postLen < 0) postLen = curLenWords;
        break;
      case mSetNote:
        if (!afterOn) preOnSetNote = macro[i].data[0];
        break;
      case mOn:
        if (!hadFirstOn) {
          firstOnStart = curStart;
          firstOnLen = curLenWords;
          hadFirstOn = true;
        
        }
        onStart = curStart;
        onLen = curLenWords;
        gotAny = true;
        afterOn = true;
        break;
      case mSetLoop: {
        int refLen = (postLen > 0) ? postLen : onLen;
        int offsetBytes = (macro[i].data[1]<<8)|(macro[i].data[2]);
        info.loopStart = offsetBytes;
        info.loopLen = refLen * 2 - offsetBytes;
        if (info.loopLen < 2) info.loopLen = 2;
        info.hasSetLoop = true;
        break;
      }
      case mOneShot:
        info.oneShot = true;
        break;
      case mWaitUp:
        afterWaitUp = true;
        break;
      case mLoopUp:
        if (macro[i].data[0] == 0)
          afterWaitUp = true;
        break;
      case mAddNote: {
        int val = (signed char)macro[i].data[0];
        if (afterOn)
          addNotePostOn = val;
        else
          addNotePreOn = val;
        break;
      }
      case mAddBegin:
        if (afterOn && !gotAddBegin) {
          int amt = (signed short)((macro[i].data[1]<<8)|macro[i].data[2]);
          if (amt < 0) {
            info.negSweep = true;
            info.negSweepAmt = amt;
            info.negSweepHp = macro[i].data[0];
          }
          if (amt > 0) {
            info.sweepAmt = amt;
            int frames = 0;
            bool hitInfiniteLoop = false;
            int effectiveHp = macro[i].data[0];
            for (int j = i + 1; j < 256; j++) {
              if (macro[j].op == mWait) {
                int w = (macro[j].data[0]<<16)|(macro[j].data[1]<<8)|macro[j].data[2];
                frames += w + 1;
              } else if (macro[j].op == mWaitSample) {
                int w = macro[j].data[0];
                frames += (w > 0) ? w : 1;
              } else if (macro[j].op == mLoop) {
                int loopCount = macro[j].data[0];
                int loopTarget = macro[j].data[2];
                if (loopCount > 0 && loopTarget < j) {
                  int bodyFrames = 0;
                  for (int k = loopTarget; k < j; k++) {
                    if (macro[k].op == mWait) {
                      int w = (macro[k].data[0]<<16)|(macro[k].data[1]<<8)|macro[k].data[2];
                      bodyFrames += w + 1;
                    } else if (macro[k].op == mWaitSample) {
                      int w = macro[k].data[0];
                      bodyFrames += (w > 0) ? w : 1;
                    }
                  }
                  int alreadyCounted = 0;
                  for (int k = (loopTarget > i + 1 ? loopTarget : i + 1); k < j; k++) {
                    if (macro[k].op == mWait) {
                      int w = (macro[k].data[0]<<16)|(macro[k].data[1]<<8)|macro[k].data[2];
                      alreadyCounted += w + 1;
                    } else if (macro[k].op == mWaitSample) {
                      int w = macro[k].data[0];
                      alreadyCounted += (w > 0) ? w : 1;
                    }
                  }
                  frames -= alreadyCounted;
                  frames += bodyFrames * loopCount;
                } else if (loopCount == 0) {
                  hitInfiniteLoop = true;
                  break;
                }
              } else if (macro[j].op == mAddBegin) {
                int amt2 = (signed short)((macro[j].data[1]<<8)|macro[j].data[2]);
                if (info.sweepAmt2 == 0 && amt2 != 0 && amt2 != amt) {
                  info.sweepAmt2 = amt2;
                  info.sweepReverseFrame = frames;
                } else if (amt2 == amt) {
                  effectiveHp = macro[j].data[0];
                }
              } else if (macro[j].op == mOneShot || macro[j].op == mStop) {
                break;
              }
            }
            if (hitInfiniteLoop) {
              info.sweepAmt = 0;
              info.sweepAmt2 = 0;
              info.sweepReverseFrame = 0;
            } else {
              if (frames <= 0) frames = 40;
              info.sweepFrames = frames;
              info.sweepHalfPeriod = effectiveHp;
            }
          }
          gotAddBegin = true;
        }
        break;
      case mStop:
        goto done;
      default:
        break;
    }
  }
done:
  if (gotAny) {
    if (postStart >= 0 && postLen > 0) {
      info.start = postStart;
      info.len = postLen * 2;
      /* Multi-phase macro with mSetNote before mOn: the pre-mOn sample
       * plays at a fixed pitch on the Amiga hardware channel.  Store the
       * attack info so the simulation can emit it on an overflow channel.
       * Use the FIRST mOn's sample, not the last (which may point to the
       * body sample after a second mOn). */
      if (preOnSetNote >= 0) {
        info.atkStart   = firstOnStart;
        info.atkLen     = firstOnLen * 2;
        info.setNoteVal = preOnSetNote;
      }
    } else {
      info.start = onStart;
      info.len = onLen * 2;
    }
  } else {
    info.start = curStart;
    info.len = curLenWords * 2;
  }
  /* Prefer post-mOn mAddNote for sustain (runs during playback); fall back
   * to pre-mOn if no post-mOn mAddNote exists. */
  info.addNote = (addNotePostOn != -999) ? addNotePostOn : addNotePreOn;
  return info;
}

/*
 * Convert a TFMX per-tick envelope rate to an XM per-row volume slide speed.
 *
 * TFMX applies envAmt every (envTimeC+1) frames.  XM volume slide runs on
 * ticks 1..(xmSpeed-1), giving (xmSpeed-1) applications per row.  Combining
 * into a single fraction avoids premature integer truncation:
 *   speed = round( envAmt * xmSpeed / ((timeC+1) * (xmSpeed-1)) )
 * Result is clamped to XM's 1-15 range.
 */
static int calcEnvSpeed(int envAmt, int envTimeC, int xmSpeed) {
  int xmTicksPerRow = (xmSpeed > 1) ? (xmSpeed - 1) : 1;
  int denom = (envTimeC + 1) * xmTicksPerRow;
  int speed = (envAmt * xmSpeed + denom / 2) / denom;
  if (speed < 1) speed = 1;
  if (speed > 15) speed = 15;
  return speed;
}

/*
 * Extract effect parameters (volume, vibrato, envelopes) from a TFMX macro.
 *
 * TFMX macros are linear sequences of opcodes executed per-tick. This function
 * walks the macro once and translates the relevant opcodes into XM-compatible
 * effect values. Key mapping decisions:
 *
 *   mAddVol / mSetVol → effective volume (applied in post-processing)
 *   mVibrato          → XM effect 4xy (vibrato speed x, depth y)
 *   mEnv before mWaitUp → sustain envelope (XM volume slide 6x/7x)
 *   mEnv after mWaitUp  → release envelope (triggered by pKeyUp)
 *   mWait              → sustain delay in rows
 *
 * For multi-phase macros (e.g., loud attack then quiet sustain via mSetVol
 * after mOn), the PRE-mOn volume command defines the note-on volume in XM.
 * Post-mOn mSetVol/mAddVol only override when both are in the same phase.
 */
static MacroEffectInfo getMacroEffects(const TFMXMacroData allMacros[128][256],
                                       int macroIdx, int xmSpeed) {
  MacroEffectInfo fx = {-999, -1, 0, 0, false, 0, 0, 0, 0, 0, 0, false, 0, 0, 0, 0, 0, 0};
  bool foundVib = false;
  bool afterOn = false;
  bool afterWaitUp = false;
  bool foundSustainEnv = false;
  bool foundReleaseEnv = false;
  int waitBeforeFirstEnv = 0;
  bool hasPreOnVol = false;
  bool hadNewSampleSinceWait = false;
  int waitsSinceOn = 0;
  const TFMXMacroData* macro = allMacros[macroIdx];

  for (int i = 0; i < 256; i++) {
    switch (macro[i].op) {
      case mAddVol:
        if (!afterWaitUp) {
          fx.addVol = (signed char)macro[i].data[2];
          fx.setVol = -1;
          if (!afterOn) hasPreOnVol = true;
        }
        break;
      case mSetVol:
        if (!afterWaitUp) {
          fx.setVol = macro[i].data[2];
          fx.addVol = -999;
          if (!afterOn) hasPreOnVol = true;
        }
        break;
      case mSetBegin:
        break;
      case mOn:
        afterOn = true;
        break;
      case mWait:
        if (afterOn) waitsSinceOn++;
        if (afterOn && !foundSustainEnv && !afterWaitUp) {
          int w = (macro[i].data[0]<<16)|(macro[i].data[1]<<8)|(macro[i].data[2]);
          waitBeforeFirstEnv += w + 1;
        }
        break;
      case mWaitUp:
        afterWaitUp = true;
        break;
      case mLoopUp:
        /* mLoopUp(0,...) is an unconditional jump gated by keyon — it acts
         * as a "sustain hold" loop that only breaks when pKeyUp fires.
         * Everything after it is effectively the release phase. */
        if (macro[i].data[0] == 0)
          afterWaitUp = true;
        break;
      case mWaitSample:
        break;
      case mVibrato: {
        if (!foundVib) {
          int timeC = macro[i].data[0] & 0xfe;
          int amt = macro[i].data[2];
          if (timeC > 0 && amt > 0) {
            int xs = 32 / timeC;
            if (xs < 1) xs = 1; if (xs > 15) xs = 15;
            int yd = (amt * timeC + 62) / 63;
            if (yd < 1) yd = 1; if (yd > 15) yd = 15;
            fx.vibSpeed = xs;
            fx.vibDepth = yd;
            fx.vibAfterRelease = afterWaitUp;
          }
          foundVib = true;
        }
        break;
      }
      case mEnv: {
        int envAmt = macro[i].data[0];
        int envTimeC = macro[i].data[1];
        int envTarget = macro[i].data[2];
        int speed = calcEnvSpeed(envAmt, envTimeC, xmSpeed);

        if (!foundSustainEnv && !afterWaitUp) {
          /* Sustain envelope: applied while keyon, before pKeyUp */
          fx.sustainDelay = (waitBeforeFirstEnv + xmSpeed - 1) / xmSpeed;
          if (fx.sustainDelay > 16) fx.sustainDelay = 16;
          fx.sustainTarget = envTarget;
          if (envTarget == 0) {
            fx.sustainSlide = -speed;
          } else {
            int initVol = (fx.addVol != -999) ? fx.addVol + 24 :
                          (fx.setVol >= 0) ? fx.setVol : 48;
            fx.sustainSlide = (envTarget > initVol) ? speed : -speed;
          }
          foundSustainEnv = true;
        } else if (foundSustainEnv && !afterWaitUp && !foundReleaseEnv
                   && envTarget == 0 && fx.sustainTarget > 0) {
          /* Two-stage sustain: first stage fades to non-zero target,
           * second stage continues to 0.  Store second stage as a release
           * that auto-starts when the first target is reached. */
          fx.releaseSlide = -speed;
          fx.releaseTarget = 0;
          fx.autoRelease = true;
          foundReleaseEnv = true;
        } else if (afterWaitUp && !foundReleaseEnv) {
          /* First release envelope (may target a non-zero volume) */
          fx.releaseTarget = envTarget;
          fx.releaseSlide = (envTarget == 0) ? -speed : -speed;
          if (envTarget > 0) {
            int initVol = (fx.addVol != -999) ? fx.addVol + 24 :
                          (fx.setVol >= 0) ? fx.setVol : 48;
            fx.releaseSlide = (envTarget < initVol) ? -speed : speed;
          }
          foundReleaseEnv = true;
        } else if (afterWaitUp && foundReleaseEnv && envTarget == 0) {
          /* Second release envelope: slow fade from releaseTarget to 0 */
          fx.releaseSlide2 = -speed;
        }
        break;
      }
      case mCont:
        /* mCont after mOn: continue execution in a different macro at a
         * given step.  Used for wavetable-attack → sustain transitions.
         * We follow the redirect so that envelope/volume extraction
         * picks up the sustain macro's parameters. */
        if (afterOn) {
          int target = macro[i].data[0];
          int step   = (macro[i].data[1] << 8) | macro[i].data[2];
          if (target >= 0 && target < 128 && step >= 0 && step < 256) {
            macro = allMacros[target];
            i = step - 1;
          }
        }
        break;
      case mStop: goto efxdone;
      default: break;
    }
  }
efxdone:
  /* If the macro has a sustain fade to 0 but no explicit release envelope,
   * reuse the sustain slide as the release — otherwise notes hang forever. */
  if (fx.releaseSlide == 0 && foundSustainEnv && fx.sustainSlide < 0
      && fx.sustainTarget == 0) {
    fx.releaseSlide = fx.sustainSlide;
  }
  /* If the first release stage stops at a non-zero volume but there is no
   * explicit second stage, continue fading at the same speed so the note
   * actually silences (XM has no "hold at release level" concept). */
  if (fx.releaseSlide != 0 && fx.releaseTarget > 0 && fx.releaseSlide2 == 0) {
    fx.releaseSlide2 = fx.releaseSlide;
  }

  /* Detect stutter pattern: mSetVol(0) → mWait → mSetVol(V>0) → mWait → mLoop.
   * Common in TFMX for tremolo-like pulsing pads (e.g., T2 World5 macros 68/69).
   * We record the timing so Phase 2 can emit alternating volumes. */
  {
    int totalFramesBefore = 0;
    bool pastOn = false;
    for (int i = 0; i < 255; i++) {
      if (macro[i].op == mOn) pastOn = true;
      if (!pastOn && macro[i].op == mWait) {
        int w = (macro[i].data[0]<<16)|(macro[i].data[1]<<8)|macro[i].data[2];
        totalFramesBefore += w + 1;
      }
      if (pastOn && macro[i].op == mWait) {
        int w = (macro[i].data[0]<<16)|(macro[i].data[1]<<8)|macro[i].data[2];
        totalFramesBefore += w + 1;
      }
      /* Pattern: [mAddBegin] mSetVol(0) mWait mSetVol(V) mWait mLoop */
      if (macro[i].op == mSetVol && macro[i].data[2] == 0 && pastOn) {
        int addBeginStep = 0;
        if (i > 0 && macro[i-1].op == mAddBegin)
          addBeginStep = (macro[i-1].data[0]<<16)|(macro[i-1].data[1]<<8)|macro[i-1].data[2];
        int j = i + 1;
        if (j < 255 && macro[j].op == mWait) {
          int offW = (macro[j].data[0]<<16)|(macro[j].data[1]<<8)|macro[j].data[2];
          int offFrames = offW + 1;
          j++;
          if (j < 255 && macro[j].op == mSetVol && macro[j].data[2] > 0) {
            int onVol = macro[j].data[2];
            j++;
            if (j < 255 && macro[j].op == mWait) {
              int onW = (macro[j].data[0]<<16)|(macro[j].data[1]<<8)|macro[j].data[2];
              int onFrames = onW + 1;
              j++;
              if (j < 256 && macro[j].op == mLoop) {
                fx.stutterOffFrames = offFrames;
                fx.stutterOnFrames  = onFrames;
                fx.stutterVol       = onVol;
                fx.stutterDelay     = totalFramesBefore;
                fx.stutterFadeCycles = 0;
                if (addBeginStep > 0) {
                  int smpBytes = 0;
                  for (int k = 0; k < i; k++) {
                    if (macro[k].op == mSetLen)
                      smpBytes = ((macro[k].data[1]<<8)|macro[k].data[2]) * 2;
                  }
                  if (smpBytes > 0)
                    fx.stutterFadeCycles = smpBytes / addBeginStep;
                }
                /* The stutter loop's mSetVol commands overwrote addVol/setVol.
                 * Restore the initial volume from before the stutter pattern
                 * so note trigger volume is computed from mAddVol, not mSetVol. */
                for (int k = 0; k < i; k++) {
                  if (macro[k].op == mAddVol) {
                    fx.addVol = (signed char)macro[k].data[2];
                    fx.setVol = -1;
                  } else if (macro[k].op == mSetVol) {
                    fx.setVol = macro[k].data[2];
                    fx.addVol = -999;
                  }
                }
                break;
              }
            }
          }
        }
      }
      if (macro[i].op == mStop) break;
    }
  }

  /* Detect macro-internal retrigger: mOff + mOn inside a mLoop body.
   * TFMX macros use this to create repeating pluck effects (e.g. T1 leads).
   * The retrigger interval = total mWait frames in the loop body + 1. */
  {
    const TFMXMacroData* origMacro = allMacros[macroIdx];
    bool seenOn = false;
    for (int i = 0; i < 256; i++) {
      if (origMacro[i].op == mStop) break;
      if (origMacro[i].op == mOn) seenOn = true;
      if (seenOn && origMacro[i].op == mLoop) {
        int loopTarget = origMacro[i].data[2];
        if (loopTarget < 0 || loopTarget >= i) continue;
        bool hasOff = false, hasOn = false;
        int waitFrames = 0;
        for (int j = loopTarget; j < i; j++) {
          if (origMacro[j].op == mOff || origMacro[j].op == mOffReset) hasOff = true;
          if (origMacro[j].op == mOn) hasOn = true;
          if (origMacro[j].op == mWait) {
            int w = (origMacro[j].data[0]<<16)|(origMacro[j].data[1]<<8)|origMacro[j].data[2];
            waitFrames += w + 1;
          }
        }
        if (hasOff && hasOn && waitFrames > 0)
          fx.retriggerFrames = waitFrames;
        break;
      }
    }
  }

  return fx;
}

/*
 * Delta-encode 8-bit sample data for the XM format with gain compensation.
 * XM stores samples as deltas: each byte = (current - previous) & 0xFF.
 *
 * A 2x gain compensates for the Amiga-vs-XM mixing architecture:
 * the Amiga mixes 2 channels per stereo side additively (~25% headroom
 * per channel), while XM players normalize across 8+ channels (~3-6%).
 * Boosting the sample data is the only way to restore perceived loudness,
 * since XM pattern volume already caps at 64.
 */
static void deltaEncode8(const signed char* src, int len, unsigned char* dst) {
  int prev = 0;
  for (int i = 0; i < len; i++) {
    int s = (unsigned char)src[i];
    dst[i] = (unsigned char)((s - prev) & 0xff);
    prev = s;
  }
}



/*
 * Read a 4-byte TFMX pattern entry from the raw mdat buffer.
 *
 * TFMX pattern entries are packed as:
 *   byte 0: note (or command opcode if >= 0xF0)
 *   byte 1: instrument / parameter
 *   byte 2: high nibble = volume (0-15), low nibble = channel (0-7)
 *   byte 3: detune / sub-parameter
 *
 * Returns a synthetic pEnd if the offset is out of bounds (safety fallback).
 */
static TFMXPatData readPatEntry(const unsigned char* mdatBuf, size_t mdatSize, int fileOffset) {
  TFMXPatData item;
  if (fileOffset < 0 || fileOffset + 4 > (int)mdatSize) {
    item.note = 0xF0;
    item.ins = 0; item.vol = 0; item.chan = 0; item.detune = 0;
    return item;
  }
  item.note   = mdatBuf[fileOffset];
  item.ins    = mdatBuf[fileOffset + 1];
  item.vol    = (mdatBuf[fileOffset + 2] >> 4) & 0xf;
  item.chan    = mdatBuf[fileOffset + 2] & 0xf;
  item.detune = mdatBuf[fileOffset + 3];
  return item;
}

/* Read big-endian u32 from raw buffer. */
static unsigned int readBE32(const unsigned char* buf, int off) {
  return ((unsigned int)buf[off]<<24)|((unsigned int)buf[off+1]<<16)|
         ((unsigned int)buf[off+2]<<8)|(unsigned int)buf[off+3];
}

/*
 * Main conversion entry point.
 *
 * Loads the TFMX mdat and smpl files, simulates playback to build XM pattern
 * data, then writes a complete XM file with 128 instruments (one per macro).
 *
 * @param mdatPath  Path to the TFMX module data file (header + patterns + macros + orders)
 * @param smplPath  Path to the TFMX sample data file (raw 8-bit signed PCM)
 * @param outPath   Destination path for the output XM file
 * @param subsong   Subsong index (0-31) to convert — TFMX modules can contain
 *                  multiple songs, each with its own start/end order row and speed
 */
bool convertToXM(const char* mdatPath, const char* smplPath, const char* outPath, int subsong, PanPreset pan) {
  int xmErrors = 0;
  FILE* f;
  TFMXHeader head;
  int patPoint[128], macroPoint[128];
  TFMXOrders track[128][8];
  TFMXMacroData macro[128][256];
  signed char* smpl = NULL;
  size_t smplLen = 0;

  /* Stereo panning values for XM effect 8xx (0x00=full left, 0x80=center, 0xFF=full right).
   * Amiga mode reproduces the original hard-panned Amiga sound.
   * Soft narrows the image slightly for speakers; Headphone narrows further. */
  unsigned char panL, panR;
  switch (pan) {
    case PAN_AMIGA:    panL = 0x00; panR = 0xFF; break;
    case PAN_HEADPHONE: panL = 0x30; panR = 0xD0; break;
    default:           panL = 0x10; panR = 0xF0; break;
  }
  const char* panNames[] = {"Soft","Amiga","Headphone"};
  printf("Panning preset: %s (L=0x%02X R=0x%02X)\n", panNames[pan], panL, panR);

  /* Open conversion.log in the same directory as the output XM file (append mode).
   * All detailed diagnostic output (macro info, sample stats, note events) goes
   * here to keep the console output concise. */
  char logPath[1024];
  {
    const char* lastSlash = strrchr(outPath, '/');
    if (lastSlash) {
      size_t dirLen = (size_t)(lastSlash - outPath);
      if (dirLen >= sizeof(logPath)) dirLen = sizeof(logPath) - 1;
      memcpy(logPath, outPath, dirLen);
      logPath[dirLen] = '\0';
    } else {
      logPath[0] = '.'; logPath[1] = '\0';
    }
    strncat(logPath, "/conversion.log", sizeof(logPath) - strlen(logPath) - 1);
  }

  /* Log rotation: if conversion.log exceeds ~1 MB, rotate to numbered archives.
   * conversion.log → conversion_old_01.log, shifting existing archives up.
   * Maximum 9 archives; the oldest (conversion_old_09.log) is deleted. */
  {
    FILE* probe = fopen(logPath, "rb");
    if (probe) {
      fseek(probe, 0, SEEK_END);
      long sz = ftell(probe);
      fclose(probe);
      if (sz >= 1024 * 1024) {
        char dirBuf[1024];
        const char* ls = strrchr(logPath, '/');
        if (ls) {
          size_t dl = (size_t)(ls - logPath);
          if (dl >= sizeof(dirBuf)) dl = sizeof(dirBuf) - 1;
          memcpy(dirBuf, logPath, dl);
          dirBuf[dl] = '\0';
        } else {
          dirBuf[0] = '.'; dirBuf[1] = '\0';
        }
        char oldPath[1024], newPath[1024];
        snprintf(oldPath, sizeof(oldPath), "%s/conversion_old_09.log", dirBuf);
        remove(oldPath);
        for (int n = 8; n >= 1; n--) {
          snprintf(oldPath, sizeof(oldPath), "%s/conversion_old_%02d.log", dirBuf, n);
          snprintf(newPath, sizeof(newPath), "%s/conversion_old_%02d.log", dirBuf, n + 1);
          rename(oldPath, newPath);
        }
        snprintf(newPath, sizeof(newPath), "%s/conversion_old_01.log", dirBuf);
        rename(logPath, newPath);
      }
    }
  }

  FILE* logFile = fopen(logPath, "a");
  if (logFile) {
    time_t now = time(NULL);
    struct tm* t = localtime(&now);
    char timeBuf[64];
    strftime(timeBuf, sizeof(timeBuf), "%Y-%m-%d %H:%M:%S", t);
    fprintf(logFile, "\n========================================\n");
    fprintf(logFile, "[%s] Convert: %s → %s  (subsong %d)\n", timeBuf, mdatPath, outPath, subsong);
    fprintf(logFile, "Panning: %s (L=0x%02X R=0x%02X)\n", panNames[pan], panL, panR);
    fprintf(logFile, "========================================\n");
  }

  /* Load smpl */
  f = fopen(smplPath, "rb");
  if (!f) {
    perror(smplPath);
    if (logFile) fclose(logFile);
    return false;
  }
  fseek(f, 0, SEEK_END);
  smplLen = ftell(f);
  fseek(f, 0, SEEK_SET);
  smpl = (signed char*)malloc(smplLen);
  if (!smpl) {
    fclose(f);
    if (logFile) fclose(logFile);
    return false;
  }
  fread(smpl, 1, smplLen, f);
  fclose(f);

  /* Load entire mdat file into a flat buffer.
   * We need random access because TFMX pattern commands (pGsPt, pJump) can
   * reference arbitrary file offsets, not just sequential pattern entries. */
  f = fopen(mdatPath, "rb");
  if (!f) {
    perror(mdatPath);
    free(smpl);
    if (logFile) fclose(logFile);
    return false;
  }
  fseek(f, 0, SEEK_END);
  size_t mdatSize = ftell(f);
  fseek(f, 0, SEEK_SET);
  unsigned char* mdatBuf = (unsigned char*)malloc(mdatSize);
  if (!mdatBuf) {
    fclose(f);
    free(smpl);
    if (logFile) fclose(logFile);
    return false;
  }
  fread(mdatBuf, 1, mdatSize, f);
  fclose(f);

  /* Parse TFMX header (big-endian Amiga format → host byte order).
   * The header contains seek offsets for orders, patterns, and macros,
   * plus per-subsong metadata (start row, end row, speed). */
  if (mdatSize < 512) {
    free(mdatBuf);
    free(smpl);
    if (logFile) fclose(logFile);
    return false;
  }
  memcpy(&head, mdatBuf, sizeof(head));
  head.ordSeek = ntohl(head.ordSeek);
  head.patSeek = ntohl(head.patSeek);
  head.macroSeek = ntohl(head.macroSeek);
  for (int i = 0; i < 32; i++) {
    head.songStart[i] = ntohs(head.songStart[i]);
    head.songEnd[i] = ntohs(head.songEnd[i]);
    head.songSpeed[i] = ntohs(head.songSpeed[i]);
  }
  /* Fallback offsets for modules that leave these fields zero */
  if (head.ordSeek == 0) head.ordSeek = 0x800;
  if (head.patSeek == 0) head.patSeek = 0x400;
  if (head.macroSeek == 0) head.macroSeek = 0x600;

  /* Pattern pointer table: 128 big-endian u32 offsets into the mdat file.
   * Each points to the first 4-byte entry of a TFMX pattern. */
  for (int i = 0; i < 128; i++) {
    int off = head.patSeek + i * 4;
    if (off + 4 <= (int)mdatSize)
      patPoint[i] = (int)readBE32(mdatBuf, off);
    else
      patPoint[i] = 0;
  }

  /* Macro pointer table: 128 big-endian u32 offsets to macro definitions.
   * Each macro is a sequence of 4-byte opcodes (op + 3 data bytes). */
  for (int i = 0; i < 128; i++) {
    int off = head.macroSeek + i * 4;
    if (off + 4 <= (int)mdatSize)
      macroPoint[i] = (int)readBE32(mdatBuf, off);
    else
      macroPoint[i] = 0;
  }

  /* Order table: 128 rows × 8 tracks. Each entry is 2 bytes: (pattern, transposition).
   * The pattern byte selects a TFMX pattern (0-127), or a special value:
   *   0x80 = continue (don't restart this track)
   *   0xFE = channel-off (silence the channel indicated by 'trans')
   *   0xFF = track inactive / stop */
  for (int r = 0; r < 128; r++) {
    for (int t = 0; t < 8; t++) {
      int off = head.ordSeek + (r * 8 + t) * 2;
      if (off + 2 <= (int)mdatSize) {
        track[r][t].pat = mdatBuf[off];
        track[r][t].trans = mdatBuf[off + 1];
      } else {
        track[r][t].pat = 0xFF;
        track[r][t].trans = 0;
      }
    }
  }

  /* Load macro opcodes from the mdat buffer.
   * Each macro is terminated by mStop (opcode 7). We read up to 256 steps. */
  for (int i = 0; i < 128; i++) {
    if (macroPoint[i] <= 0 || macroPoint[i] + 4 > (int)mdatSize) {
      macro[i][0].op = mStop;
      continue;
    }
    int off = macroPoint[i];
    int s = 0;
    while (s < 256 && off >= 0 && off + 4 <= (int)mdatSize) {
      macro[i][s].op = mdatBuf[off];
      macro[i][s].data[0] = mdatBuf[off + 1];
      macro[i][s].data[1] = mdatBuf[off + 2];
      macro[i][s].data[2] = mdatBuf[off + 3];
      if (macro[i][s].op == mStop) break;
      s++;
      off += 4;
    }
  }

  int startRow = head.songStart[subsong];
  int endRow = head.songEnd[subsong];
  if (startRow < 0) startRow = 0;
  if (startRow > 127) startRow = 127;
  if (endRow > 127) endRow = 127;
  if (endRow < startRow) endRow = startRow;

  bool multimode = false;
  for (int r = startRow; r <= endRow; r++) {
    if (track[r][0].pat == 0xEF && track[r][0].trans == 0xFE) {
      int cmd = (track[r][1].pat << 8) | track[r][1].trans;
      if (cmd == 3) { multimode = true; break; }
    }
  }
  (void)multimode; /* detection logged in summary line */

  /* Volume mapping: wraps scaleVol() so all volume conversions in this
   * function go through a single point (easy to swap for per-mode curves). */
  auto volMap = [&](int v) -> int {
    (void)multimode;
    return scaleVol(v);
  };

  /* Open output XM */
  FILE* out = fopen(outPath, "wb");
  if (!out) {
    perror(outPath);
    free(mdatBuf);
    free(smpl);
    if (logFile) fclose(logFile);
    return false;
  }

  /* XM pre-header: "Extended Module: " (17) + name 20 + 0x1A + tracker 20 + version 2 + header_size 4 */
  fwrite("Extended Module: ", 1, 17, out);
  char xmName[21];
  memset(xmName, 0, sizeof(xmName));
  if (multimode)
    snprintf(xmName, 21, "TFMX[7v] Subsong %02d", subsong);
  else
    snprintf(xmName, 21, "TFMX Subsong %02d", subsong);
  fwrite(xmName, 1, 20, out);
  fputc(0x1A, out);
  memset(xmName, 0, 20);
  snprintf(xmName, 21, "tfmxplay %s", TFMXPLAY_VERSION);
  fwrite(xmName, 1, 20, out);
  writeU16(out, 0x0104);
  writeU32(out, 276);

  /* ===================================================================
   * PHASE 1: Simulate TFMX playback tick-by-tick.
   *
   * TFMX doesn't have fixed "rows" — it's driven by pWait commands that
   * pause individual tracks for N ticks. We model this by running a loop
   * where each iteration = one XM row = one tick. Note/volume events are
   * placed into a flat "bigGrid" array that we later slice into 64-row
   * XM patterns.
   * =================================================================== */
  const int ROWS_PER_PAT = 64;
  const int MAX_TOTAL_ROWS = ROWS_PER_PAT * 256;
  const int BASE_CHANS = 8;    /* original TFMX channels */
  const int OVERFLOW_CHANS = 4; /* attack transients: only Amiga ch 0–3 → XM ch 9–12 */
  const int STUTTER_CHANS = 4; /* pulsing pads: Amiga ch 0–3 → XM ch 13–16 */
  int numChans = BASE_CHANS + OVERFLOW_CHANS + STUTTER_CHANS;

  XMCell* bigGrid = (XMCell*)calloc((size_t)MAX_TOTAL_ROWS * numChans, sizeof(XMCell));
  if (!bigGrid) { fclose(out); free(mdatBuf); free(smpl); if (logFile) fclose(logFile); return false; }

  /*
   * Per-track simulation state. TFMX has 8 "tracks" that run patterns
   * independently. Each track reads its pattern by stepping through 4-byte
   * entries at (baseOff + pos*4) in the mdat buffer.
   */
  struct TrackState {
    int baseOff;       /* absolute file offset of current pattern start; -1 = inactive */
    int pos;           /* current step index within the pattern */
    int tim;           /* ticks remaining until this track processes next command */
    signed char trans; /* transposition in semitones (from order table) */
    int loopCount;     /* loop counter for pLoop command */
    int savedBaseOff;  /* return address for pGsPt (GoSub Pattern) */
    int savedPos;      /* return step for pGsPt */
    bool inGosub;      /* true while executing a pattern subroutine */
  };
  TrackState ts[8];
  for (int t = 0; t < 8; t++) {
    ts[t].baseOff = -1; ts[t].pos = -1;
    ts[t].tim = 0x7fffffff; ts[t].inGosub = false;
  }

  /*
   * Per-channel pattern-level envelope state (driven by pEnve commands).
   * Unlike macro envelopes (which are per-instrument), pEnve is a pattern
   * command that directly manipulates a channel's volume over time.
   * Used e.g. for chord pads fading out: pEnve sets amt/timeReset/target,
   * then each tick we step vol toward target.
   */
  struct ChanEnv {
    int amt;       /* volume change per step */
    int timeReset; /* ticks between volume steps */
    int timeC;     /* countdown to next step */
    int target;    /* target volume (0-64) */
    bool active;   /* true if envelope is running */
    int vol;       /* current volume (0-64), initialized from effective note volume */
  };
  ChanEnv chanEnv[8];
  bool chanAlive[8];             /* true while a channel has an audible note playing */
  bool stutterUsed[4] = {false, false, false, false}; /* tracks which Amiga ch use stutter overflow */
  int lastEnveVol[8];            /* last scaled volume written by pEnve, for computing slides */
  bool chanPortaActive[8];       /* true while tone portamento (3xx) is active on a channel */
  int  chanPortaParam[8];        /* current portamento speed for 3xx continuation rows */
  int  chanLastMacro[8];         /* last instrument (macro index) triggered on each channel */
  for (int c = 0; c < 8; c++) {
    chanEnv[c].active = false;
    chanAlive[c] = false;
    lastEnveVol[c] = 0;
    chanPortaActive[c] = false;
    chanPortaParam[c] = 0;
    chanLastMacro[c] = -1;
  }

  int totalXmRows = 0;          /* running count of XM rows written so far */
  int ordRowToXmRow[128];       /* maps TFMX order row index → first XM row */
  memset(ordRowToXmRow, -1, sizeof(ordRowToXmRow));
  int loopJumpXmRow = -1;       /* XM row where EFFE cmd 1 (loop) was encountered */
  int loopJumpTargetXmRow = -1; /* XM row the loop jumps back to */

  /* TFMX speed = ticks per row. The header stores (speed - 1). */
  int xmSpeed = head.songSpeed[subsong] + 1;
  if (xmSpeed < 1) xmSpeed = 6;
  if (xmSpeed > 31) xmSpeed = 31;

  /* CIA-based BPM.  Default eClocks=14318 → 125 BPM (PAL 50 Hz).
   * EFFE 0003 (timeshare) overrides this before simulation starts. */
  int eClocks = 14318;
  int xmBPM   = 125;

  /* Pre-scan order table for EFFE speed changes that occur before any
   * pattern data.  This ensures macroFX envelope calculations use the
   * actual playback speed rather than the (potentially placeholder) header value. */
  for (int r = startRow; r <= endRow; r++) {
    if (track[r][0].pat == 0xEF && track[r][0].trans == 0xFE) {
      int cmd = (track[r][1].pat << 8) | track[r][1].trans;
      if (cmd == 2) {
        int w2 = (track[r][2].pat << 8) | track[r][2].trans;
        int newSpeed = (w2 & 0xFF) + 1;
        if (newSpeed >= 1 && newSpeed <= 31) xmSpeed = newSpeed;
      }
    } else {
      break;
    }
  }

  /* XM volume slides need at least 2 ticks per row (tick 0 = set,
   * ticks 1..N = slide).  If the TFMX song uses speed 0 (= every VBL),
   * bump to speed 2 and double BPM to keep the same playback rate.
   * Track the doubling so that later CIA/EFFE BPM overrides are also doubled. */
  bool speedWasDoubled = false;
  if (xmSpeed == 1) {
    xmSpeed = 2;
    xmBPM = 250;
    speedWasDoubled = true;
  }

  /* Resolve mCont chains: if a macro's first action is mCont(target),
   * the effective macro is the target. Build a redirect table so that
   * sample info, effects, and hasOn checks use the resolved macro. */
  /* Resolve mCont chains: if a macro's first opcode is mCont(target),
   * the effective instrument is the target macro.  This is common for
   * wavetable-attack → sustain-sample transitions.  Build a redirect
   * table so sample info, effects, and hasOn checks use the resolved macro. */
  int macroContTarget[128];
  /* mSKey (split keyboard): stores split note and alternate target.
   * When a macro starts with mSKey, notes >= sKeySplit use sKeyHigh target;
   * lower notes use the default macroContTarget (sKeyLow). */
  int sKeySplit[128];
  int sKeyLow[128];
  int sKeyHigh[128];
  for (int i = 0; i < 128; i++) {
    macroContTarget[i] = -1;
    sKeySplit[i] = -1;
    sKeyLow[i] = -1;
    sKeyHigh[i] = -1;
    int skeyOffset = 0;
    bool hasSKey = false;
    for (int j = 0; j < 256; j++) {
      if (macro[i][j].op == mSKey) {
        sKeySplit[i] = macro[i][j].data[0];
        skeyOffset = macro[i][j].data[2];
        hasSKey = true;
        continue;
      }
      if (macro[i][j].op == mCont) {
        int target = macro[i][j].data[0];
        if (target >= 0 && target < 128 && target != i) {
          if (hasSKey && sKeyLow[i] < 0) {
            sKeyLow[i] = target;
            macroContTarget[i] = target;
            if (skeyOffset <= 1) {
              sKeyHigh[i] = target;
            }
          } else if (hasSKey && sKeyHigh[i] < 0) {
            sKeyHigh[i] = target;
          } else {
            macroContTarget[i] = target;
          }
        }
        if (!hasSKey) break;
        continue;
      }
      if (macro[i][j].op == mOn || macro[i][j].op == mStop) break;
    }
    /* Second pass: detect mCont AFTER mOn for wavetable-sweep macros.
     * Some macros play a tiny single-cycle waveform (<=128 bytes) with
     * mAddBegin sweep as an attack, then mCont to a different macro with
     * the actual sustain sample (e.g., sampled string).  The target macro
     * provides the sustained sound that the listener perceives.
     * Only applies when no pre-mOn mCont was found and the source sample
     * is small (indicating wavetable synthesis, not a real sample). */
    if (macroContTarget[i] < 0 && sKeySplit[i] < 0) {
      bool pastOn = false;
      int srcLen = 0;
      for (int j = 0; j < 256; j++) {
        if (macro[i][j].op == mSetLen && !pastOn)
          srcLen = ((macro[i][j].data[1] << 8) | macro[i][j].data[2]) * 2;
        if (macro[i][j].op == mOn) pastOn = true;
        if (pastOn && macro[i][j].op == mCont) {
          int target = macro[i][j].data[0];
          if (target >= 0 && target < 128 && target != i &&
              srcLen > 0 && srcLen <= 128)
            macroContTarget[i] = target;
          break;
        }
        if (macro[i][j].op == mStop) break;
      }
    }
  }

  /* Pre-compute macro effects for all 128 instruments.
   * Needed before simulation so pEnve can initialize chanEnv.vol to the
   * correct effective volume (accounting for mAddVol/mSetVol).
   * For mCont macros, use the target's effects but keep any pre-mCont
   * volume/oneShot overrides from the source macro. */
  MacroEffectInfo macroFX[128];
  bool macroHasOn[128];
  for (int i = 0; i < 128; i++) {
    int resolved = (macroContTarget[i] >= 0) ? macroContTarget[i] : i;
    macroFX[i] = getMacroEffects(macro, resolved, xmSpeed);
    /* Merge pre-mCont overrides (e.g. mAddVol before mCont) */
    if (macroContTarget[i] >= 0) {
      for (int j = 0; j < 256; j++) {
        if (macro[i][j].op == mAddVol)
          macroFX[i].addVol = (signed char)macro[i][j].data[2];
        else if (macro[i][j].op == mSetVol)
          macroFX[i].setVol = macro[i][j].data[2];
        else if (macro[i][j].op == mCont) break;
        else if (macro[i][j].op == mStop) break;
      }
    }
    macroHasOn[i] = false;
    for (int j = 0; j < 256; j++) {
      if (macro[resolved][j].op == mOn) { macroHasOn[i] = true; break; }
      if (macro[resolved][j].op == mStop) break;
    }
  }

  /* Pre-compute sample info for all macros (needed for attack overflow).
   * For macros with mSetNote before mOn, the pre-mOn sample is a fixed-pitch
   * attack transient.  We map each such macro to an attack XM instrument
   * stored in the upper instrument slots (macro + 65).
   * For mCont macros, use the resolved target's sample info. */
  MacroSampleInfo macroSI[128];
  MacroSampleInfo macroSrcSI[128]; /* source macro's sample info (before mCont resolve) */
  int atkInstMap[128];  /* atkInstMap[m] = XM instrument number (1-based) for attack; 0 = none */
  for (int i = 0; i < 128; i++) {
    macroSrcSI[i] = getMacroSample(macro[i]);
    int resolved = (macroContTarget[i] >= 0) ? macroContTarget[i] : i;
    macroSI[i] = getMacroSample(macro[resolved]);
    /* mOneShot in source macro overrides oneShot flag */
    if (macroContTarget[i] >= 0) {
      for (int j = 0; j < 256; j++) {
        if (macro[i][j].op == mOneShot) { macroSI[i].oneShot = true; break; }
        if (macro[i][j].op == mCont || macro[i][j].op == mStop) break;
      }
    }
    (void)macroFX[i];
    atkInstMap[i] = 0;
    if (macroSI[i].setNoteVal < 0 || macroSI[i].atkLen < 32 || i + 64 >= 128)
      continue;
    int as = macroSI[i].atkStart, al = macroSI[i].atkLen;
    int ss = macroSI[i].start, sl = macroSI[i].len;
    /* Overflow attack only for real short transients.  Loose rules used to put
     * huge wrong slices on ch 9–10 (garbage / wrong timbre). */
    const int MAX_ATK_BYTES = 8192;
    if (al > MAX_ATK_BYTES)
      continue;
    if (as == ss && al >= sl)
      continue; /* not shorter than sustain — no separate click */
    if (as == ss && al * 4 > sl * 3)
      continue; /* attack > ~75% of sustain: treat as false positive */
    if (as != ss && al > sl && sl >= 256)
      continue; /* attack block longer than sustain body — bogus */
    atkInstMap[i] = i + 64 + 1;
  }

  /* Map attack instrument indices back to their parent macro so Phase 2
   * can look up the correct volume effects (addVol/setVol). */
  int atkParentMacro[128];
  memset(atkParentMacro, -1, sizeof(atkParentMacro));
  for (int i = 0; i < 128; i++) {
    if (atkInstMap[i] > 0) {
      int atkIdx = atkInstMap[i] - 1;
      if (atkIdx >= 0 && atkIdx < 128)
        atkParentMacro[atkIdx] = i;
    }
  }

  /* Diagnostic: dump full volume/envelope info for active macros → log */
  if (logFile) {
    fprintf(logFile, "\n--- Volume Diagnostics (full) ---\n");
    for (int i = 0; i < 128; i++) {
      if (macroFX[i].addVol != -999 || macroFX[i].setVol >= 0 ||
          macroSI[i].sweepAmt != 0) {
        MacroEffectInfo& ef = macroFX[i];
        fprintf(logFile, "  Macro %3d: addVol=%4d  setVol=%3d  sweep=%d"
               "  susSlide=%d susTarget=%d susDelay=%d"
               "  relSlide=%d relTarget=%d relSlide2=%d autoRel=%d"
               "  stutOn=%d stutOff=%d stutVol=%d  retrig=%d\n",
               i, ef.addVol, ef.setVol, macroSI[i].sweepAmt,
               ef.sustainSlide, ef.sustainTarget, ef.sustainDelay,
               ef.releaseSlide, ef.releaseTarget, ef.releaseSlide2,
               ef.autoRelease ? 1 : 0,
               ef.stutterOnFrames, ef.stutterOffFrames, ef.stutterVol,
               ef.retriggerFrames);
      }
    }
    fprintf(logFile, "---\n\n");
  }

  /* Diagnostic: dump raw macro steps for key macros → log */
  if (logFile) {
    static const char* opNames[] = {
      "mOffReset","mOn","mSetBegin","mSetLen","mWait","mLoop","mCont","mStop",
      "mAddNote","mSetNote","mReset","mPorta","mVibrato","mAddVol","mSetVol",
      "mEnv","mLoopUp","mAddBegin","mAddLen","mOff","mWaitUp","mGoSub","mRet",
      "mSetPeriod","mSetLoop","mOneShot","mWaitSample","mRand","mSKey","mSVol",
      "mAddVolNote","mSetPrevNote","mSignal","mPlayMacro"
    };
    fprintf(logFile, "--- Raw Macro Steps (key macros) ---\n");
    int dumpMacros[] = {53, 54, 62, 63, -1};
    for (int dm = 0; dumpMacros[dm] >= 0; dm++) {
      int m = dumpMacros[dm];
      fprintf(logFile, "  Macro %d:\n", m);
      for (int s = 0; s < 30; s++) {
        int op = macro[m][s].op;
        const char* name = (op < 34) ? opNames[op] : "???";
        fprintf(logFile, "    %2d: %-12s (%02X %02X %02X)\n", s, name,
               macro[m][s].data[0], macro[m][s].data[1], macro[m][s].data[2]);
        if (op == mStop || op == mLoop) break;
      }
    }
    fprintf(logFile, "---\n\n");
  }

  /* Diagnostic: attack overflow instruments → log */
  if (logFile) {
    fprintf(logFile, "--- Attack Overflow Instruments ---\n");
    for (int i = 0; i < 128; i++) {
      if (atkInstMap[i] > 0)
        fprintf(logFile, "  Macro %3d -> Attack Inst %d (atkStart=%d atkLen=%d smpStart=%d smpLen=%d setNote=%d)\n",
               i, atkInstMap[i], macroSI[i].atkStart, macroSI[i].atkLen,
               macroSI[i].start, macroSI[i].len, macroSI[i].setNoteVal);
    }
    fprintf(logFile, "---\n\n");
  }

  /* Per-instrument stereo tracking: count how many notes each instrument
   * plays on left vs right channels.  Used after simulation to decide
   * whether panning can be baked into the sample header (single side)
   * or needs per-note 8xx effects (played on both sides). */
  int instPlayLeft[128];
  int instPlayRight[128];
  memset(instPlayLeft, 0, sizeof(instPlayLeft));
  memset(instPlayRight, 0, sizeof(instPlayRight));

  /* Track the first XM note played for each instrument.  This lets the
   * sweep-bake code (Phase 4) estimate the Amiga DMA playback rate so
   * the baked waveform length matches the original sweep duration. */
  int instFirstXmNote[128];
  memset(instFirstXmNote, -1, sizeof(instFirstXmNote));

  /*
   * Initialize track states from an order table row.
   * Each of the 8 tracks reads its pattern assignment and transposition.
   * Special pattern values are handled: 0x80 (continue), 0xFE (channel off),
   * 0xFF (inactive). For normal patterns (0-127), the track resets to the
   * first step of the referenced pattern.
   */
  auto initOrderRow = [&](int ordRow) {
    for (int t = 0; t < 8; t++) {
      unsigned char p = track[ordRow][t].pat;
      ts[t].trans = (signed char)track[ordRow][t].trans;
      if (p == 0x80) continue;
      if (p == 0xFE) {
        int ch = track[ordRow][t].trans;
        if (ch >= 0 && ch < BASE_CHANS && totalXmRows < MAX_TOTAL_ROWS) {
          bigGrid[totalXmRows * numChans + ch].vol = 0x10;
          chanAlive[ch] = false;
          lastEnveVol[ch] = 0;
        }
        ts[t].baseOff = -1;
        ts[t].tim = 0x7fffffff;
        continue;
      }
      if (p >= 0x80) {
        ts[t].baseOff = -1;
        ts[t].tim = 0x7fffffff;
        continue;
      }
      ts[t].baseOff = patPoint[p];
      ts[t].pos = -1;
      ts[t].tim = -1;
      ts[t].loopCount = 0;
      ts[t].inGosub = false;
    }
  };

  /* Main simulation loop: iterate over order table rows.
   * Each order row assigns patterns to the 8 tracks. We process one order
   * row at a time; within each row, we tick until any track hits pEnd
   * (which means this order row is finished). */
  for (int ordRow = startRow; ordRow <= endRow && totalXmRows < MAX_TOTAL_ROWS; ordRow++) {
    /* Special order row commands: pat=0xEF, trans=0xFE on track 0 signals
     * a meta-command. cmd=0: stop song, cmd=1: jump to row, cmd=2: tempo. */
    if (track[ordRow][0].pat == 0xEF && track[ordRow][0].trans == 0xFE) {
      int cmd = (track[ordRow][1].pat << 8) | track[ordRow][1].trans;
      int w2  = (track[ordRow][2].pat << 8) | track[ordRow][2].trans;
      int w3  = (track[ordRow][3].pat << 8) | track[ordRow][3].trans;
      if (cmd == 0) break;
      if (cmd == 1) {
        int jumpTarget = w2 & 0xFF;
        if (jumpTarget <= ordRow) {
          loopJumpXmRow = totalXmRows > 0 ? totalXmRows - 1 : 0;
          loopJumpTargetXmRow = ordRowToXmRow[jumpTarget];
          break;
        }
        ordRow = jumpTarget - 1;
      }
      if (cmd == 2) {
        /* Speed change: w2 = new prescale (ticks-1 per row) */
        int newSpeed = (w2 & 0xFF) + 1;
        if (newSpeed == 1) { newSpeed = 2; speedWasDoubled = true; }
        if (newSpeed >= 1 && newSpeed <= 31) {
          if (totalXmRows > 0 && newSpeed != xmSpeed) {
            XMCell* cell = &bigGrid[(totalXmRows - 1) * numChans];
            for (int c = 0; c < numChans; c++) {
              if (cell[c].fx == 0) { cell[c].fx = 0x0F; cell[c].param = (unsigned char)newSpeed; break; }
            }
          }
          xmSpeed = newSpeed;
          if (newSpeed > 2) speedWasDoubled = false;
        }
        /* Optional CIA override in w3 */
        if (!(w3 & 0xF200) && (w3 & 0x1FF) > 0xF) {
          eClocks = 0x1B51F8 / (w3 & 0x1FF);
          int newBPM = (int)(1790456.0 / eClocks + 0.5);
          if (speedWasDoubled) newBPM *= 2;
          if (newBPM < 32) newBPM = 32; if (newBPM > 255) newBPM = 255;
          if (totalXmRows > 0 && newBPM != xmBPM) {
            XMCell* cell = &bigGrid[(totalXmRows - 1) * numChans];
            for (int c = 0; c < numChans; c++) {
              if (cell[c].fx == 0) { cell[c].fx = 0x0F; cell[c].param = (unsigned char)newBPM; break; }
            }
          }
          xmBPM = newBPM;
        }
      }
      if (cmd == 3) {
        /* Timeshare (7V mode): w3 low byte adjusts CIA timing */
        if (!(w3 & 0x8000)) {
          int x = (signed char)(w3 & 0xFF);
          if (x < -0x20) x = -0x20;
          eClocks = (14318 * (x + 100)) / 100;
          int newBPM = (int)(1790456.0 / eClocks + 0.5);
          if (speedWasDoubled) newBPM *= 2;
          if (newBPM < 32) newBPM = 32; if (newBPM > 255) newBPM = 255;
          if (totalXmRows > 0 && newBPM != xmBPM) {
            XMCell* cell = &bigGrid[(totalXmRows - 1) * numChans];
            for (int c = 0; c < numChans; c++) {
              if (cell[c].fx == 0) { cell[c].fx = 0x0F; cell[c].param = (unsigned char)newBPM; break; }
            }
          }
          xmBPM = newBPM;
          (void)x; /* timeshare detected */
        }
      }
      /* cmd == 4 (fade) is handled implicitly by per-channel volume envelopes */
      continue;
    }

    ordRowToXmRow[ordRow] = totalXmRows;
    if (logFile) {
      fprintf(logFile, "  ORD %3d → xmRow %5d  pats:", ordRow, totalXmRows);
      for (int t = 0; t < 8; t++)
        fprintf(logFile, " %02X/%+d", track[ordRow][t].pat, (signed char)track[ordRow][t].trans);
      fprintf(logFile, "\n");
    }
    initOrderRow(ordRow);

    /* Tick loop for this order row. Each iteration = one XM row.
     * All 8 tracks are ticked in sequence. When a track's timer expires,
     * it reads pattern commands until it either hits a pWait (sets a new
     * timer) or a pEnd (signals end of this order row section). */
    bool rowDone = false;
    int safety = 16384;
    while (!rowDone && totalXmRows < MAX_TOTAL_ROWS && --safety > 0) {
      bool anyActive = false;
      for (int t = 0; t < 8 && !rowDone; t++) {
        if (ts[t].baseOff < 0) continue;
        if (--ts[t].tim >= 0) { anyActive = true; continue; }

        anyActive = true;
        bool getMeOut = false;
        int innerSafety = 512;
        while (!getMeOut && !rowDone && --innerSafety > 0) {
          ts[t].pos++;
          TFMXPatData item = readPatEntry(mdatBuf, mdatSize, ts[t].baseOff + ts[t].pos * 4);

          switch (item.note) {
            case pEnd:
              /* End of pattern. If inside a gosub (pGsPt), return to the
               * calling pattern. Otherwise, signal that this order row is done. */
              if (ts[t].inGosub) {
                ts[t].baseOff = ts[t].savedBaseOff;
                ts[t].pos = ts[t].savedPos;
                ts[t].inGosub = false;
              } else {
                rowDone = true; getMeOut = true;
              }
              break;
            case pStop:
              /* Permanently stop this track for the rest of the song */
              ts[t].baseOff = -1;
              ts[t].tim = 0x7fffffff; getMeOut = true;
              break;
            case pWait:
              /* Pause this track for item.ins ticks (reference player DoTrack
               * case 3: PWait=x.b.b1; return(0)). */
              ts[t].tim = item.ins; getMeOut = true;
              break;
            case pLoop: {
              /* Loop within pattern. ins=0: unconditional jump to step.
               * ins>0: loop (ins+1) times, then fall through. */
              int target = ((item.vol << 12) | (item.chan << 8) | item.detune) - 1;
              if (item.ins == 0) {
                ts[t].pos = target;
              } else {
                if (ts[t].loopCount == 0) ts[t].loopCount = item.ins + 1;
                if (--ts[t].loopCount != 0) ts[t].pos = target;
              }
              break;
            }
            case pJump:
              /* Jump to another pattern (ins) at step offset (vol:chan:detune).
               * Unlike pGsPt, this does NOT save a return address. */
              if (item.ins < 128)
                ts[t].baseOff = patPoint[item.ins];
              ts[t].pos = ((item.vol << 12) | (item.chan << 8) | item.detune) - 1;
              ts[t].loopCount = 0;
              break;
            case pGsPt: {
              /* GoSub Pattern: save current position, then jump to another
               * pattern as a subroutine. pEnd in the target will return here.
               * Only saves if not already inside a gosub (no nesting). */
              if (!ts[t].inGosub) {
                ts[t].savedBaseOff = ts[t].baseOff;
                ts[t].savedPos = ts[t].pos;
                ts[t].inGosub = true;
              }
              if (item.ins < 128)
                ts[t].baseOff = patPoint[item.ins];
              ts[t].pos = ((item.vol << 12) | (item.chan << 8) | item.detune) - 1;
              ts[t].loopCount = 0;
              break;
            }
            case pRoPt:
              /* Return from Pattern subroutine (inverse of pGsPt) */
              if (ts[t].inGosub) {
                ts[t].baseOff = ts[t].savedBaseOff;
                ts[t].pos = ts[t].savedPos;
                ts[t].inGosub = false;
              }
              break;
            case pEnve: {
              int ch = item.chan;
              if (ch >= 0 && ch < BASE_CHANS) {
                chanEnv[ch].amt = item.ins;
                chanEnv[ch].timeReset = item.vol + 1;
                chanEnv[ch].timeC = item.vol + 1;
                chanEnv[ch].target = item.detune;
                chanEnv[ch].active = true;
              }
              break;
            }
            case pKeyUp: {
              /* Release → 97 → Phase 2.  NEVER schedule 97 beyond row+1: a KeyUp
               * many rows ahead would fire after the *next* note on that channel
               * and corrupt envelopes (wrong subsong / „passt nicht“). */
              int ch = item.chan;
              if (ch < 0 || ch >= BASE_CHANS || totalXmRows >= MAX_TOTAL_ROWS)
                break;
              XMCell* here = &bigGrid[totalXmRows * numChans + ch];
              if (here->note >= 1 && here->note < 97) {
                int nr = totalXmRows + 1;
                if (nr < MAX_TOTAL_ROWS) {
                  XMCell* nxt = &bigGrid[nr * numChans + ch];
                  if (nxt->note == 0 || nxt->note == 97)
                    nxt->note = 97;
                }
              } else {
                here->note = 97;
              }
              break;
            }
            case pPPat: {
              /* PPat (0xFB): redirect another track to play a different pattern.
               * item.ins  = pattern number to start
               * item.chan  = target track (0-7) to redirect
               * item.detune = transposition for the new pattern
               * The calling track continues executing its own commands. */
              int targetTrack = item.chan;
              if (targetTrack >= 0 && targetTrack < 8 && item.ins < 128) {
                ts[targetTrack].baseOff = patPoint[item.ins];
                ts[targetTrack].trans = (signed char)item.detune;
                ts[targetTrack].pos = -1;
                ts[targetTrack].tim = 0;
                ts[targetTrack].loopCount = 0;
                ts[targetTrack].inGosub = false;
              }
              break;
            }
            case pStCu:
              /* Stop Custom: freeze this track permanently (like pStop but
               * keeps the track "alive" — just sets an infinite timer). */
              ts[t].tim = 0x7fffffff;
              getMeOut = true;
              break;
            case pNOP: break;
            default:
              if (item.note >= 0xf0) break;
              if ((item.note & 0xc0) == 0xc0) {
                /* Portamento: slide from current pitch to the target note.
                 * TFMX uses proportional portamento (period *= (256±rate)/256
                 * every portaReset frames). We approximate with XM effect 3xx
                 * (linear tone portamento).
                 *
                 * Direction fix: in TFMX, NotePort sets DestPeriod directly
                 * from notevals[target] WITHOUT applying the macro's addNote,
                 * but the current pitch already HAS addNote baked in.  XM's
                 * tone portamento applies relativeNote to both source and
                 * target equally, which can reverse the slide direction.
                 * We compensate by subtracting the instrument's addNote from
                 * the target note so the XM player arrives at the correct
                 * un-shifted pitch after adding relativeNote back. */
                int ch = item.chan;
                if (ch < 0 || ch >= BASE_CHANS) ch = 0;
                int rawNote = (item.note & 0x3f) + ts[t].trans;
                int addNoteCompensation = 0;
                if (chanLastMacro[ch] >= 0 && chanLastMacro[ch] < 128)
                  addNoteCompensation = macroSI[chanLastMacro[ch]].addNote;
                int xmNote = rawNote - addNoteCompensation + 25;
                if (xmNote < 1) xmNote = 1;
                if (xmNote > 96) xmNote = 96;
                int portaRate  = item.detune;
                int portaReset = item.ins;
                if (portaReset < 1) portaReset = 1;
                int xmParam = (portaRate * 5 + portaReset * 2)
                              / (portaReset * 4);
                if (xmParam < 1) xmParam = 1;
                if (xmParam > 0xFF) xmParam = 0xFF;
                if (totalXmRows < MAX_TOTAL_ROWS) {
                  XMCell* cell = &bigGrid[totalXmRows * numChans + ch];
                  cell->note = (unsigned char)xmNote;
                  cell->inst = 0;
                  cell->vol  = 0;
                  cell->fx = 3;
                  cell->param = (unsigned char)xmParam;
                }
                chanPortaActive[ch] = true;
                chanPortaParam[ch] = xmParam;
                break;
              }

              {
                /* Regular note event. TFMX note = 6-bit pitch + transposition.
                 * The +25 offset maps TFMX notes to XM so that the effective
                 * playback frequency (after per-instrument relativeNote from
                 * mAddNote) matches the original TFMX NTSC period within 0.1%.
                 * Derivation: TFMX freq = 3579545 * 2^(N/12) / 2033.83
                 *             XM freq   = 8363 * 2^((xmNote-49)/12)
                 *             Solving: xmNote = tfmxNote + 25 */
                int rawNote = (item.note & 0x3f) + ts[t].trans;
                int xmNote = rawNote + 25;
                if (xmNote < 1) xmNote = 1;
                if (xmNote > 96) xmNote = 96;
                int ch = item.chan;
                if (ch < 0 || ch >= BASE_CHANS) ch = 0;
                if (totalXmRows < MAX_TOTAL_ROWS) {
                  XMCell* cell = &bigGrid[totalXmRows * numChans + ch];

                  
                  /* mSKey: redirect instrument based on note vs split point */
                  int effectiveIns = item.ins;
                  if (effectiveIns >= 0 && effectiveIns < 128 &&
                      sKeySplit[effectiveIns] >= 0) {
                    int tfmxNote = item.note & 0x3f;
                    if (tfmxNote >= sKeySplit[effectiveIns] &&
                        sKeyHigh[effectiveIns] >= 0)
                      effectiveIns = sKeyHigh[effectiveIns];
                    else if (sKeyLow[effectiveIns] >= 0)
                      effectiveIns = sKeyLow[effectiveIns];
                  }
                  bool emptyMacro = (effectiveIns >= 0 && effectiveIns < 128 &&
                                     !macroHasOn[effectiveIns]);
                  int useInst = effectiveIns;
                  if (emptyMacro && chanLastMacro[ch] >= 0 &&
                      chanLastMacro[ch] < 128 && macroHasOn[chanLastMacro[ch]])
                    useInst = chanLastMacro[ch];
                  if (!emptyMacro || useInst != item.ins) {
                    cell->note = (unsigned char)xmNote;
                    cell->inst = (unsigned char)(useInst + 1);
                    int v = item.vol;
                    int idx = useInst;
                    int prelimVol = v * 4;
                    if (prelimVol > 64) prelimVol = 64;
                    cell->vol = (unsigned char)(0x10 + prelimVol);
                    if (cell->vol > 0x50) cell->vol = 0x50;
                    int effVol = prelimVol;
                    if (idx >= 0 && idx < 128) {
                      MacroEffectInfo& ef = macroFX[idx];
                      if (ef.setVol >= 0) effVol = ef.setVol;
                      else if (ef.addVol != -999)     effVol = ef.addVol + v * 3;
                      else                             effVol = v * 3;
                      if (effVol > 64) effVol = 64;
                      if (effVol < 0) effVol = 0;
                    }
                    chanEnv[ch].vol = effVol;
                    chanEnv[ch].active = false;
                    chanAlive[ch] = true;
                    chanPortaActive[ch] = false;
                    chanLastMacro[ch] = useInst;
                    lastEnveVol[ch] = volMap(chanEnv[ch].vol);

                    if (useInst >= 0 && useInst < 128) {
                      bool isR = multimode ? (ch==1||ch==2||ch==6||ch==7)
                                           : (bool)((ch & 1) ^ ((ch & 2) >> 1));
                      if (isR) instPlayRight[useInst]++;
                      else     instPlayLeft[useInst]++;
                      if (instFirstXmNote[useInst] < 0)
                        instFirstXmNote[useInst] = xmNote;
                    }

                    int macIdx = useInst;
                    if (macIdx >= 0 && macIdx < 128 && atkInstMap[macIdx] > 0 &&
                        ch >= 0 && ch < OVERFLOW_CHANS) {
                      int ovfCh = BASE_CHANS + ch;
                      int atkNote = macroSI[macIdx].setNoteVal + 25;
                      if (atkNote >= 1 && atkNote <= 96) {
                        XMCell* atkCell = &bigGrid[totalXmRows * numChans + ovfCh];
                        atkCell->note = (unsigned char)atkNote;
                        atkCell->inst = (unsigned char)atkInstMap[macIdx];
                        /* Only halve volume when attack and sustain overlap
                         * (same sample region) — those sound doubled.  Distinct
                         * attacks (e.g. snare click) keep full volume. */
                        MacroSampleInfo& siAtk = macroSI[macIdx];
                        bool doubled = (siAtk.atkStart == siAtk.start);
                        int v = (int)cell->vol - 0x10;
                        if (v < 0) v = 0;
                        if (doubled) v = v / 2;
                        atkCell->vol  = (unsigned char)(0x10 + v);
                        int atkIdx = atkInstMap[macIdx] - 1;
                        if (atkIdx >= 0 && atkIdx < 128) {
                          bool isR = multimode ? (ch==1||ch==2||ch==6||ch==7)
                                               : (bool)((ch & 1) ^ ((ch & 2) >> 1));
                          if (isR) instPlayRight[atkIdx]++;
                          else     instPlayLeft[atkIdx]++;
                        }
                      }
                    }

                    if (macIdx >= 0 && macIdx < 128 &&
                        macroFX[macIdx].stutterOnFrames > 0 &&
                        ch >= 0 && ch < STUTTER_CHANS) {
                      int stCh = BASE_CHANS + OVERFLOW_CHANS + ch;
                      XMCell* stCell = &bigGrid[totalXmRows * numChans + stCh];
                      stCell->note = cell->note;
                      stCell->inst = cell->inst;
                      stCell->vol  = cell->vol;
                      cell->note = 0;
                      cell->inst = 0;
                      cell->vol  = 0;
                      stutterUsed[ch] = true;
                      if (item.ins >= 0 && item.ins < 128) {
                        bool isR = multimode ? (ch==1||ch==2||ch==6||ch==7)
                                             : (bool)((ch & 1) ^ ((ch & 2) >> 1));
                        if (isR) instPlayRight[item.ins]++;
                        else     instPlayLeft[item.ins]++;
                      }
                    } else if (macIdx >= 0 && macIdx < 128 &&
                               macroFX[macIdx].stutterOnFrames == 0 &&
                               ch >= 0 && ch < STUTTER_CHANS &&
                               stutterUsed[ch]) {
                      int stCh = BASE_CHANS + OVERFLOW_CHANS + ch;
                      XMCell* stCell = &bigGrid[totalXmRows * numChans + stCh];
                      if (stCell->note == 0)
                        stCell->vol = 0x10;
                    }
                  }
                }
                /* TFMX note types: bits 7:6 = 10 (0x80-0xBF) or note=0x7F
                 * indicate "note + wait" — the detune field becomes the wait count. */
                if ((item.note & 0xc0) == 0x80 || item.note == 0x7f) {
                  ts[t].tim = item.detune;
                  getMeOut = true;
                }
              }
              break;
          }
        }
      }
      /* Continue active portamento (XM effect 300) on empty cells.
       * The initial 3xx was placed when the TFMX portamento note fired;
       * here we repeat effect 300 on every subsequent row so the XM
       * player keeps sliding toward the target pitch. */
      if (totalXmRows < MAX_TOTAL_ROWS && !rowDone) {
        for (int c = 0; c < BASE_CHANS; c++) {
          if (!chanPortaActive[c]) continue;
          XMCell* cell = &bigGrid[totalXmRows * numChans + c];
          if (cell->note == 0 && cell->inst == 0 && cell->fx == 0) {
            cell->fx = 3;
            cell->param = 0;
          }
        }
      }

      /* Apply active pEnve (pattern-level envelopes).
       * In real TFMX hardware, envelopes run per-frame (~50Hz), while
       * pattern commands run per-tick (every xmSpeed frames). So the
       * envelope fires xmSpeed times per simulation tick (= 1 XM row).
       * We loop xmSpeed times here to match the original fade speed.
       *
       * Skip when rowDone: the pEnd tick is a boundary artefact — the
       * next order row's first tick will also target the same XM row.
       * Firing pEnve for both would double the fade speed. */
      if (totalXmRows < MAX_TOTAL_ROWS && !rowDone) {
        for (int c = 0; c < BASE_CHANS; c++) {
          if (!chanEnv[c].active) continue;
          int envFrames = (xmSpeed * 2 + 2) / 3;
          /* Akkord-Pads (pEnve → 0): schneller ausfaden pro Tick */
          if (chanEnv[c].target == 0 && chanEnv[c].vol > 0)
            envFrames = xmSpeed;
          if (envFrames < 1) envFrames = 1;
          if (envFrames > xmSpeed) envFrames = xmSpeed;
          for (int fr = 0; fr < envFrames && chanEnv[c].active; fr++) {
            if (--chanEnv[c].timeC <= 0) {
              chanEnv[c].timeC = chanEnv[c].timeReset;
              if (chanEnv[c].vol < chanEnv[c].target)
                chanEnv[c].vol = (chanEnv[c].vol + chanEnv[c].amt < chanEnv[c].target)
                                  ? chanEnv[c].vol + chanEnv[c].amt : chanEnv[c].target;
              else if (chanEnv[c].vol > chanEnv[c].target)
                chanEnv[c].vol = (chanEnv[c].vol - chanEnv[c].amt > chanEnv[c].target)
                                  ? chanEnv[c].vol - chanEnv[c].amt : chanEnv[c].target;
              if (chanEnv[c].vol == chanEnv[c].target) {
                chanEnv[c].active = false;
                if (chanEnv[c].target == 0) {
                  chanAlive[c] = false;
                  chanPortaActive[c] = false;
                }
              }
            }
          }
          /* Skip pEnve writes when the channel has no active sound.
           * chanAlive[c] is set true on note-on and cleared when pEnve
           * reaches target 0, so it accurately tracks whether an audible
           * sample is still playing — regardless of how many rows have
           * passed since the last note event. */
          if (!chanAlive[c]) {
            XMCell* cell = &bigGrid[totalXmRows * numChans + c];
            if (cell->note == 0 && cell->inst == 0 && cell->vol == 0 &&
                lastEnveVol[c] > 0) {
              cell->vol = 0x10;
              lastEnveVol[c] = 0;
            }
            continue;
          }
          XMCell* cell = &bigGrid[totalXmRows * numChans + c];
          int v = chanEnv[c].vol;
          if (v < 0) v = 0;
          if (v > 64) v = 64;
          int newScaled = volMap(v);
          int delta = lastEnveVol[c] - newScaled;
          int slideTicks = (xmSpeed > 1) ? (xmSpeed - 1) : 1;
          if (cell->note == 0 && cell->inst == 0 && cell->vol == 0) {
            if (delta > 0) {
              int speed = (delta + slideTicks - 1) / slideTicks;
              if (speed >= 1 && speed <= 15) {
                cell->vol = (unsigned char)(0x60 + speed);
                lastEnveVol[c] -= speed * slideTicks;
                if (lastEnveVol[c] < 0) lastEnveVol[c] = 0;
              } else {
                cell->vol = (unsigned char)(0x10 + newScaled);
                lastEnveVol[c] = newScaled;
              }
            } else if (delta < 0) {
              int speed = ((-delta) + slideTicks - 1) / slideTicks;
              if (speed >= 1 && speed <= 15) {
                cell->vol = (unsigned char)(0x70 + speed);
                lastEnveVol[c] += speed * slideTicks;
                if (lastEnveVol[c] > 64) lastEnveVol[c] = 64;
              } else {
                cell->vol = (unsigned char)(0x10 + newScaled);
                lastEnveVol[c] = newScaled;
              }
            }
          } else if (cell->note > 0 && cell->note < 97 && cell->inst > 0
                     && cell->fx == 0 && delta != 0) {
            if (delta > 0) {
              int speed = (delta + slideTicks - 1) / slideTicks;
              if (speed < 1) speed = 1;
              if (speed > 15) speed = 15;
              cell->fx = 0x0A;
              cell->param = (unsigned char)speed;
              lastEnveVol[c] -= speed * slideTicks;
              if (lastEnveVol[c] < 0) lastEnveVol[c] = 0;
            } else {
              int speed = ((-delta) + slideTicks - 1) / slideTicks;
              if (speed < 1) speed = 1;
              if (speed > 15) speed = 15;
              cell->fx = 0x0A;
              cell->param = (unsigned char)(speed << 4);
              lastEnveVol[c] += speed * slideTicks;
              if (lastEnveVol[c] > 64) lastEnveVol[c] = 64;
            }
          }
        }
      }
      if (!anyActive) rowDone = true;
      if (!rowDone && totalXmRows < MAX_TOTAL_ROWS) totalXmRows++;
    }
  }
  if (totalXmRows < 1) totalXmRows = 1;

  /* Compute per-instrument sample panning from the tracked channel usage.
   * If an instrument always plays on same-panning channels (all-left or
   * all-right), we bake the panning into the sample header so XM sets it
   * on note trigger.  This frees the effect column for E9x, 0xy, etc.
   * Mixed instruments (played on both sides) fall back to 8xx per note. */
  /* Decide per-instrument panning strategy:
   * - All notes on one side → bake into sample header (frees effect column)
   * - Notes on both sides → mark as needing per-note 8xx pan effect */
  unsigned char instSamplePan[128];
  bool instNeedsPanFx[128];
  for (int i = 0; i < 128; i++) {
    if (instPlayRight[i] > 0 && instPlayLeft[i] == 0) {
      instSamplePan[i] = panR;
      instNeedsPanFx[i] = false;
    } else if (instPlayLeft[i] > 0 && instPlayRight[i] == 0) {
      instSamplePan[i] = panL;
      instNeedsPanFx[i] = false;
    } else if (instPlayLeft[i] > 0 && instPlayRight[i] > 0) {
      instSamplePan[i] = (instPlayLeft[i] >= instPlayRight[i]) ? panL : panR;
      instNeedsPanFx[i] = true;
    } else {
      instSamplePan[i] = 128;
      instNeedsPanFx[i] = false;
    }
  }


  /* Look up song-specific patches: name → hash → note fingerprint */
  unsigned int mHash = hashMdat(mdatBuf, mdatSize);
  unsigned int noteFP = computeNoteFingerprint(bigGrid, totalXmRows, numChans);
  const char* matchMethod = NULL;
  const SongPatch* patch = findSongPatch(mdatPath, mHash, noteFP, subsong, &matchMethod);
  printf("mdat: %s  hash: 0x%08x  noteFP: 0x%08x\n", basenameOf(mdatPath), mHash, noteFP);
  if (logFile) {
    fprintf(logFile, "\nmdat: %s  hash: 0x%08x  noteFP: 0x%08x\n", basenameOf(mdatPath), mHash, noteFP);
  }
  if (patch) {
    printf("Song patch active: %s (matched by %s)\n", patch->label, matchMethod);
    if (logFile)
      fprintf(logFile, "Song patch active: %s (matched by %s)\n", patch->label, matchMethod);
  }

  /* Per-song attack prepend table: some macros have a short attack sample
   * (different smpl region) before the sustain body, but without mSetNote.
   * The attack is prepended to the sustain only for songs/macros listed here. */
  bool atkPrepend[128]; memset(atkPrepend, 0, sizeof(atkPrepend));
  int  atkPrependOff[128]; memset(atkPrependOff, 0, sizeof(atkPrependOff));
  int  atkPrependLen[128]; memset(atkPrependLen, 0, sizeof(atkPrependLen));

  if (patch && strcmp(patch->label, "T1_Title") == 0) {
    atkPrepend[28]    = true;   /* macro 28 = drum: attack at 0x1C14, 512 bytes */
    atkPrependOff[28] = 0x1C14;
    atkPrependLen[28] = 512;

    /* Macro 23 (Inst 24): mOn/mWait opcode mismatch causes the converter to
     * misidentify the DMA-restart loop as a multi-frame sweep.  In reality,
     * all 14 iterations execute within a single Amiga frame — the macro is a
     * one-shot transient, not a sustained pad.  Disable the bogus sweep and
     * set the sample to the final (settled) DMA position. */
    if (macroSI[23].sweepAmt > 0) {
      int totalIter = macroSI[23].sweepFrames + 1;
      int settled = macroSI[23].sweepAmt * totalIter;
      macroSI[23].start += settled;
      macroSI[23].sweepAmt = 0;
      macroSI[23].sweepFrames = 0;
      macroSI[23].oneShot = true;
    }
  }

  /* T1_Title retrigger expansion: macros with internal DMA loops (mOff/mOn
   * in mLoop) produce repeated note-ons.  Only applied when the T1_Title
   * song patch is active to avoid affecting other songs. */
  if (patch && strcmp(patch->label, "T1_Title") == 0) {
    int framesPerRow = speedWasDoubled ? 1 : xmSpeed;
    for (int c = 0; c < BASE_CHANS; c++) {
      for (int row = 0; row < totalXmRows; row++) {
        XMCell* cell = &bigGrid[row * numChans + c];
        if (cell->note < 1 || cell->note >= 97 || cell->inst < 1) continue;
        int macIdx = cell->inst - 1;
        if (macIdx < 0 || macIdx >= 128) continue;
        int retrigFrames = macroFX[macIdx].retriggerFrames;
        if (retrigFrames < 2) continue;

        int retrigRows = (retrigFrames + framesPerRow / 2) / framesPerRow;
        if (retrigRows < 1) retrigRows = 1;

        int nextEventRow = totalXmRows;
        for (int r = row + 1; r < totalXmRows; r++) {
          XMCell* nc = &bigGrid[r * numChans + c];
          if ((nc->note > 0 && nc->inst > 0) || nc->note == 97) {
            nextEventRow = r; break;
          }
        }
        int gap = nextEventRow - row;
        int numRetrig = (gap - 1) / retrigRows;
        if (numRetrig < 1) { row = nextEventRow - 1; continue; }

        int baseVol = (int)cell->vol - 0x10;
        if (baseVol < 0) baseVol = 0;
        int minVol = baseVol / 5;
        if (minVol < 4) minVol = 4;

        for (int i = 1; i <= numRetrig; i++) {
          int rr = row + i * retrigRows;
          if (rr >= nextEventRow || rr >= totalXmRows) break;
          XMCell* rc = &bigGrid[rr * numChans + c];
          if (rc->note > 0 || rc->inst > 0) break;
          float t = (float)i / (float)(numRetrig + 1);
          int decVol = baseVol - (int)((float)(baseVol - minVol) * t);
          if (decVol < minVol) decVol = minVol;
          rc->note = cell->note;
          rc->inst = cell->inst;
          rc->vol  = (unsigned char)(0x10 + decVol);
          if (rc->vol > 0x50) rc->vol = 0x50;
        }
        row = nextEventRow - 1;
      }
    }
  }

  /* Orphan-note cleanup.  In multimode, notes that receive no events for
   * a long span are likely orphaned.  Insert a key-off so Phase 2 can
   * apply a natural release fade.  Song patches can override the gap. */
  {
    const int DEFAULT_GAP = multimode ? 96 : -1; /* -1 = disabled */
    for (int c = 0; c < BASE_CHANS; c++) {
      int gap = DEFAULT_GAP;
      /* Check for per-channel/instrument patch overrides */
      if (patch) {
        for (int fi = 0; patch->fixes[fi].channel != -1 || patch->fixes[fi].instrument != -1; fi++) {
          const SongFixEntry& fix = patch->fixes[fi];
          if (fix.orphanGap > 0 && (fix.channel == -1 || fix.channel == c))
            if (gap < 0 || fix.orphanGap < gap)
              gap = fix.orphanGap;
        }
      }
      if (gap < 0) continue;

      int lastEventRow = -1;
      bool noteActive = false;
      int activeInst = 0;
      for (int row = 0; row < totalXmRows; row++) {
        XMCell* cell = &bigGrid[row * numChans + c];
        bool hasEvent = (cell->note > 0 || cell->inst > 0 ||
                         cell->vol > 0  || cell->fx > 0 || cell->param > 0);
        if (cell->note > 0 && cell->note < 97 && cell->inst > 0) {
          noteActive = true;
          activeInst = cell->inst;
          lastEventRow = row;
        } else if (cell->note == 97) {
          noteActive = false;
          activeInst = 0;
          lastEventRow = row;
        } else if (hasEvent) {
          lastEventRow = row;
        } else if (noteActive && lastEventRow >= 0) {
          /* Determine effective gap for this instrument */
          int effGap = gap;
          if (patch) {
            for (int fi = 0; patch->fixes[fi].channel != -1 || patch->fixes[fi].instrument != -1; fi++) {
              const SongFixEntry& fix = patch->fixes[fi];
              if (fix.orphanGap > 0 &&
                  (fix.channel == -1 || fix.channel == c) &&
                  (fix.instrument == -1 || fix.instrument == activeInst))
                effGap = fix.orphanGap;
            }
          }
          if ((row - lastEventRow) == effGap) {
            /* Patch-specified gaps always trigger (the patch knows this
             * instrument has legitimate silences that need cutting).
             * Default gaps only trigger when no future note exists. */
            bool patchOverride = (effGap != gap);
            if (!patchOverride) {
              bool hasFutureNote = false;
              for (int frow = row + 1; frow < totalXmRows; frow++) {
                XMCell* fc = &bigGrid[frow * numChans + c];
                if (fc->note > 0 && fc->note < 97 && fc->inst > 0) {
                  hasFutureNote = true;
                  break;
                }
              }
              if (hasFutureNote) {
                lastEventRow = row;
                continue;
              }
            }
            cell->note = 97;
            noteActive = false;
            activeInst = 0;
          }
        }
      }
      /* End-of-song: if a note is still active, insert note-off on the
       * last row so it doesn't bleed across the XM loop boundary. */
      if (noteActive && totalXmRows > 1) {
        int r = totalXmRows - 1;
        XMCell* cell = &bigGrid[r * numChans + c];
        if (cell->note == 0 || cell->note == 97) {
          cell->note = 97;
          cell->vol  = 0;
        }
      }
    }
  }

  /* Song-patch: force mute on specific rows (maxVolume=0 with fromRow/toRow).
   * Writes vol=0x10 (XM "set volume 0") to kill bleeding notes at loop points. */
  if (patch) {
    for (int c = 0; c < numChans; c++) {
      for (int fi = 0; patch->fixes[fi].channel != -1 || patch->fixes[fi].instrument != -1; fi++) {
        const SongFixEntry& fix = patch->fixes[fi];
        if (fix.maxVolume != 0 || fix.fromRow < 0 || fix.toRow < 0) continue;
        if (fix.channel != -1 && fix.channel != c) continue;
        for (int row = fix.fromRow; row < fix.toRow && row < totalXmRows; row++) {
          XMCell* cell = &bigGrid[row * numChans + c];
          if (cell->note == 0 && cell->vol == 0)
            cell->vol = 0x10;
        }
      }
    }
  }

  /* Song-patch: insert note-offs for instruments with patch-specified orphan gaps.
   * Unlike the generic orphan cleanup above (which is blocked by volume events),
   * this only counts note-on events for the target instrument. */
  if (patch) {
    for (int c = 0; c < numChans; c++) {
      for (int fi = 0; patch->fixes[fi].channel != -1 || patch->fixes[fi].instrument != -1; fi++) {
        const SongFixEntry& fix = patch->fixes[fi];
        if (fix.orphanGap <= 0) continue;
        if (fix.channel != -1 && fix.channel != c) continue;
        int tgtInst = fix.instrument;
        int lastNoteRow = -1;
        bool instActive = false;
        for (int row = 0; row < totalXmRows; row++) {
          XMCell* cell = &bigGrid[row * numChans + c];
          if (cell->note > 0 && cell->note < 97 && cell->inst > 0) {
            if (tgtInst == -1 || cell->inst == tgtInst) {
              lastNoteRow = row;
              instActive = true;
            } else {
              instActive = false;
            }
          } else if (cell->note == 97) {
            instActive = false;
          } else if (instActive && lastNoteRow >= 0 &&
                     (row - lastNoteRow) == fix.orphanGap) {
            cell->note = 97;
            cell->vol  = 0;
            instActive = false;
          }
        }
      }
    }
  }

  /* Song-patch: suppress or thin-out notes in specific row ranges.
   * suppress=true blanks all matching note-ons (replaces the first with
   * a key-off to cleanly end any prior note, then removes subsequent ones).
   * minNoteGap>0 enforces a minimum distance between retriggers — notes
   * that arrive too quickly after the last kept note are blanked.
   * Both modes also clear any panning effect (8xx) on removed cells. */
  if (patch) {
    for (int c = 0; c < numChans; c++) {
      for (int fi = 0; patch->fixes[fi].channel != -1 || patch->fixes[fi].instrument != -1; fi++) {
        const SongFixEntry& fix = patch->fixes[fi];
        if (!fix.suppress && fix.minNoteGap <= 0) continue;
        if (fix.channel != -1 && fix.channel != c) continue;
        int rStart = (fix.fromRow >= 0) ? fix.fromRow : 0;
        int rEnd   = (fix.toRow >= 0) ? fix.toRow : totalXmRows;
        int lastKeptRow = -9999;
        bool inserted_off = false;
        for (int row = rStart; row < rEnd && row < totalXmRows; row++) {
          XMCell* cell = &bigGrid[row * numChans + c];
          if (cell->note > 0 && cell->note < 97 && cell->inst > 0) {
            if (fix.instrument != -1 && cell->inst != fix.instrument) continue;
            if (fix.suppress) {
              if (!inserted_off) {
                cell->note = 97;
                cell->inst = 0; cell->vol = 0;
                if (cell->fx == 8) { cell->fx = 0; cell->param = 0; }
                inserted_off = true;
              } else {
                cell->note = 0; cell->inst = 0; cell->vol = 0;
                if (cell->fx == 8) { cell->fx = 0; cell->param = 0; }
              }
            } else if (fix.minNoteGap > 0) {
              if ((row - lastKeptRow) < fix.minNoteGap) {
                cell->note = 0; cell->inst = 0; cell->vol = 0;
                if (cell->fx == 8) { cell->fx = 0; cell->param = 0; }
              } else {
                lastKeptRow = row;
              }
            }
          }
        }
      }
    }
  }

  /* ===================================================================
   * PHASE 2: Post-process the XM grid.
   *
   * During simulation, cells were written with raw pattern volumes. Now we:
   *   1. Apply effective volumes (mAddVol/mSetVol from macros)
   *   2. Set stereo panning (8xx) only for mixed-side instruments
   *   3. Place vibrato (4xy) directly on note rows when possible
   *   4. Apply sustain/release envelope fades (XM volume column 6x/7x)
   *   5. Convert pKeyUp (note 97) to release envelope triggers
   * =================================================================== */

  for (int ch = 0; ch < numChans; ch++) {
    /* Per-channel Phase 2 state, reset at each new note-on. */
    bool vibActive = false;         /* vibrato effect is currently running */
    bool vibPending = false;        /* vibrato needs to be placed on next free row */
    bool vibIsReleaseOnly = false;  /* vibrato only starts after pKeyUp */
    int envSlide = 0;               /* current volume slide speed (neg=down, pos=up) */
    int envDelayLeft = 0;           /* rows to wait before sustain slide begins */
    int releaseSlide = 0;           /* release stage 1 slide speed */
    int releaseTargetVol = 0;       /* volume where release stage 1 stops */
    int releaseSlide2 = 0;          /* release stage 2 slide (to 0) */
    bool inRelease = false;         /* true after pKeyUp or auto-release triggered */
    bool inRelease2 = false;        /* true during release stage 2 (final fade to 0) */
    bool autoRelease = false;       /* auto-start release when sustain target reached */
    int sustainTargetVol = 0;       /* volume where sustain slide stops (scaled) */
    int curVibS = 0, curVibD = 0;   /* current vibrato speed and depth (XM 4xy nibbles) */
    int runningVol = 0;             /* estimated current volume for envelope tracking */
    int stutterOn = 0, stutterOff = 0, stutterVol = 0;  /* stutter pulse timing */
    int stutterPhase = 0;           /* current position within the stutter cycle */
    int stutterDelayLeft = 0;       /* macro frames to wait before stutter starts */
    int stutterFadeCycles = 0;      /* cycles until stutter amplitude settles */
    int xmTicksPerRow = (xmSpeed > 1) ? (xmSpeed - 1) : 1; /* ticks available for slides */
    /* Determine stereo side for this channel.
     * Amiga has 4 hardware channels with fixed panning: 0=L, 1=R, 2=R, 3=L.
     * In 7-voice (multimode), the mapping is different because 8 TFMX
     * channels are multiplexed onto 4 Paula channels via timesharing. */
    bool isRight;
    if (multimode) {
      static const int ch2paula[8] = {0,1,2,3,3,0,1,2};
      isRight = (ch < 8) && (ch2paula[ch] == 1 || ch2paula[ch] == 2);
    } else {
      isRight = (ch & 1) ^ ((ch & 2) >> 1);
    }
    unsigned char panParam = isRight ? panR : panL;

    for (int row = 0; row < totalXmRows; row++) {
      XMCell* cell = &bigGrid[row * numChans + ch];



      if (cell->note > 0 && cell->note < 97 && cell->inst > 0) {
        /* --- New note: compute effective volume and set up effects --- */
        int idx = cell->inst - 1;
        if (idx >= 0 && idx < 128) {
          /* Attack instruments use their parent macro's volume effects */
          int fxIdx = (atkParentMacro[idx] >= 0) ? atkParentMacro[idx] : idx;
          MacroEffectInfo& ef = macroFX[fxIdx];

          /* Recover the original TFMX patVol (0-15) from the preliminary
           * volume we stored during simulation, then compute the effective
           * volume using the macro's volume modifiers. */
          int rawPV = ((int)cell->vol - 0x10);
          if (rawPV < 0) rawPV = 0;
          int patVol = rawPV / 4;
          int effVol;
          if (ef.setVol >= 0) effVol = ef.setVol;
          else if (ef.addVol != -999)      effVol = ef.addVol + patVol * 3;
          else                             effVol = patVol * 3;
          if (effVol > 64) effVol = 64;
          if (effVol < 0) effVol = 0;
          int scaledEV = volMap(effVol);
          /* Apply song-patch volume cap / fade if applicable */
          if (patch) {
            for (int fi = 0; patch->fixes[fi].channel != -1 || patch->fixes[fi].instrument != -1; fi++) {
              const SongFixEntry& fix = patch->fixes[fi];
              if ((fix.channel != -1 && fix.channel != ch) ||
                  (fix.instrument != -1 && fix.instrument != (idx+1)))
                continue;
              if (fix.fromRow >= 0 && row < fix.fromRow) continue;
              if (fix.toRow >= 0 && row >= fix.toRow) continue;
              if (fix.maxVolume >= 0 && scaledEV > fix.maxVolume)
                scaledEV = fix.maxVolume;
            }
          }
          cell->vol = (unsigned char)(0x10 + scaledEV);
          if (logFile) {
            static const char* noteNames[] = {"C-","C#","D-","D#","E-","F-","F#","G-","G#","A-","A#","B-"};
            int n = cell->note - 1;
            fprintf(logFile, "  NOTE ch=%d row=%d  %s%d  inst=%d vel=%d effVol=%d scaled=%d\n",
                   ch, row, noteNames[n % 12], n / 12, idx+1, patVol, effVol, scaledEV);
          }
          runningVol = scaledEV;
          /* Panning: write 8xx to effect column when free, so panning is
           * explicit per note.  Use channel-derived panParam (left/right).
           * For mixed instruments we use panParam; for single-side we also
           * use panParam so the effect column matches the channel. */
          if (cell->fx == 0) {
            cell->fx = 8;
            cell->param = panParam;
          }

          /* Vibrato: place directly on the note row when the effect column
           * is free.  If panning already occupies it, defer to the next row.
           * Release-only vibrato (vibAfterRelease) is deferred until pKeyUp. */
          curVibS = ef.vibSpeed; curVibD = ef.vibDepth;
          vibIsReleaseOnly = ef.vibAfterRelease;
          if (ef.vibSpeed > 0 && ef.vibDepth > 0 && !ef.vibAfterRelease) {
            if (cell->fx == 0) {
              cell->fx = 4;
              cell->param = (unsigned char)((curVibS << 4) | curVibD);
              vibPending = false;
            } else {
              vibPending = true;
            }
            vibActive = true;
          } else {
            vibActive = false;
            vibPending = false;
          }

          /* If Phase 1 placed a pEnve volume slide (Axy) on the note row,
           * adjust runningVol so Phase 2 tracking stays in sync. */
          if (cell->fx == 0x0A) {
            int slideDown = cell->param & 0x0F;
            int slideUp   = (cell->param >> 4) & 0x0F;
            runningVol += (slideUp - slideDown) * xmTicksPerRow;
            if (runningVol < 0) runningVol = 0;
            if (runningVol > 64) runningVol = 64;
          }

          /* Set up sustain and release envelope state for this note.
           * Scale slide speed proportionally so duration stays the same
           * despite the boosted volume.  For fade-ins (effVol=0, slide>0),
           * scale against the target volume instead. */
          envSlide = ef.sustainSlide;
          sustainTargetVol = volMap(ef.sustainTarget);
          if (envSlide != 0) {
            int refVol = effVol;
            int refScaled = scaledEV;
            if (refVol <= 0 && ef.sustainTarget > 0) {
              refVol = ef.sustainTarget;
              refScaled = sustainTargetVol;
            }
            if (refVol > 0) {
              int s = (abs(envSlide) * refScaled + refVol / 2) / refVol;
              if (s < 1) s = 1;
              if (s > 15) s = 15;
              envSlide = (envSlide < 0) ? -s : s;
            }
          }
          envDelayLeft = ef.sustainDelay;
          releaseSlide = ef.releaseSlide;
          releaseTargetVol = volMap(ef.releaseTarget);
          releaseSlide2 = ef.releaseSlide2;
          if (releaseSlide != 0 && effVol > 0) {
            int s = (abs(releaseSlide) * scaledEV + effVol / 2) / effVol;
            if (s < 1) s = 1;
            if (s > 15) s = 15;
            releaseSlide = (releaseSlide < 0) ? -s : s;
          }
          if (releaseSlide2 != 0 && effVol > 0) {
            int s = (abs(releaseSlide2) * scaledEV + effVol / 2) / effVol;
            if (s < 1) s = 1;
            if (s > 15) s = 15;
            releaseSlide2 = (releaseSlide2 < 0) ? -s : s;
          }
          inRelease = false;
          inRelease2 = false;
          autoRelease = ef.autoRelease;

          /* Set up stutter (mSetVol 0/N pulsing) */
          stutterOn = ef.stutterOnFrames;
          stutterOff = ef.stutterOffFrames;
          stutterVol = ef.stutterVol;
          stutterPhase = 0;
          stutterDelayLeft = ef.stutterDelay;
          stutterFadeCycles = ef.stutterFadeCycles;
        }
      } else if (cell->note == 97) {
        /* --- Key-up event (from pKeyUp): switch to release envelope ---
         * We clear the note-off (setting note=0) because XM's note-off
         * without a volume envelope would immediately silence the channel.
         * Instead, we apply a gradual volume slide (release envelope). */
        cell->note = 0;
        if (releaseSlide != 0) {
          int rs = releaseSlide;
          if (rs < -15) rs = -15;
          else if (rs > 15) rs = 15;
          envSlide = rs;
          envDelayLeft = 0;
          inRelease = true;
        } else if (envSlide < 0) {
          inRelease = true;
        } else {
          int s = 4;
          if (runningVol > 0) {
            s = (runningVol + xmTicksPerRow * 3 - 1) / (xmTicksPerRow * 3);
            if (s < 2) s = 2;
            if (s > 15) s = 15;
          }
          envSlide = -s;
          inRelease = true;
        }
        if (vibIsReleaseOnly && curVibS > 0 && curVibD > 0) {
          vibActive = true;
          vibPending = true;
        } else {
          vibActive = false;
          vibPending = false;
        }
      } else if (cell->note == 0 && cell->inst == 0) {
        /* --- Continuation row (no new note): apply ongoing effects --- */

        /* Vibrato: first empty row gets full 4xy, subsequent rows get 4-00 */
        if (vibPending && cell->fx == 0) {
          cell->fx = 4;
          cell->param = (unsigned char)((curVibS << 4) | curVibD);
          vibPending = false;
        } else if (vibActive && cell->fx == 0) {
          cell->fx = 4;
          cell->param = 0;
        }
        /* Stutter: alternating volume on/off from mSetVol(0)/mSetVol(N) loop.
         * Only applied on stutter overflow channels (ch >= BASE+OVF).
         * Includes linear fade-out over stutterFadeCycles. */
        if (stutterOn > 0 && stutterOff > 0 && !inRelease && cell->vol == 0
            && ch >= (BASE_CHANS + OVERFLOW_CHANS)) {
          if (stutterDelayLeft > 0) {
            stutterDelayLeft -= xmSpeed;
          } else {
            int cycle = stutterOff + stutterOn;
            int onFrames = 0;
            for (int f = 0; f < xmSpeed; f++) {
              int pos = stutterPhase % cycle;
              if (pos >= stutterOff) onFrames++;
              stutterPhase++;
            }
            int sv = volMap(stutterVol);
            /* mAddBegin shifts sample start each cycle on a looping sample →
             * timbre changes slightly but volume stays essentially constant.
             * Model as a very gentle, asymptotic fade (never reaches zero). */
            if (stutterFadeCycles > 0 && stutterPhase > 0) {
              int totalFadeFrames = stutterFadeCycles * cycle;
              int elapsed = stutterPhase;
              if (elapsed > totalFadeFrames) elapsed = totalFadeFrames;
              /* Fade to ~75 % over the fade period, then hold there */
              int minSv = (sv * 3 + 2) / 4;
              sv = minSv + ((sv - minSv) * (totalFadeFrames - elapsed)) / totalFadeFrames;
              if (sv < minSv) sv = minSv;
            }
            int avgVol = (sv * onFrames + xmSpeed / 2) / xmSpeed;
            if (avgVol > 64) avgVol = 64;
            if (avgVol < 0) avgVol = 0;
            cell->vol = (unsigned char)(0x10 + avgVol);
            runningVol = avgVol;
          }
        } else

        /* Volume envelope slide: track running volume toward target.
         * Handles both downward (envSlide < 0) and upward (envSlide > 0) slides.
         * During sustain: stop at sustainTargetVol.
         * During release stage 1: stop at releaseTargetVol, then switch to stage 2.
         * During release stage 2 (or stage 1 with target 0): slide to 0.
         * Overrides any pEnve volume left on this row — in real TFMX hardware
         * the macro envelope and pattern envelope share the same registers. */
        if (envSlide != 0) {
          if (cell->vol != 0) cell->vol = 0;
          int stopAt;
          if (inRelease && !inRelease2 && releaseTargetVol > 0)
            stopAt = releaseTargetVol;
          else if (inRelease)
            stopAt = 0;
          else
            stopAt = sustainTargetVol;

          bool moving = (envSlide < 0) ? (runningVol > stopAt)
                                       : (runningVol < stopAt);

          if (envDelayLeft > 0) {
            envDelayLeft--;
          } else if (moving) {
            int slideThisRow = envSlide * xmTicksPerRow;
            runningVol += slideThisRow;
            bool overshot = (envSlide < 0) ? (runningVol <= stopAt)
                                           : (runningVol >= stopAt);
            if (overshot) {
              if (stopAt > 0) {
                cell->vol = (unsigned char)(0x10 + stopAt);
                runningVol = stopAt;
                if (inRelease && !inRelease2 && releaseSlide2 != 0) {
                  envSlide = releaseSlide2;
                  inRelease2 = true;
                } else if (!inRelease && autoRelease && releaseSlide != 0) {
                  envSlide = releaseSlide;
                  sustainTargetVol = releaseTargetVol;
                  inRelease = true;
                } else {
                  envSlide = 0;
                }
              } else {
                cell->vol = 0x10;
                runningVol = 0;
                envSlide = 0;
                vibActive = false;
                vibPending = false;
              }
            } else {
              if (envSlide < 0)
                cell->vol = (unsigned char)(0x60 + (-envSlide));
              else if (envSlide > 0)
                cell->vol = (unsigned char)(0x70 + envSlide);
            }
          } else if (runningVol == stopAt && stopAt > 0) {
            if (inRelease && !inRelease2 && releaseSlide2 != 0) {
              envSlide = releaseSlide2;
              inRelease2 = true;
            } else if (!inRelease && autoRelease && releaseSlide != 0) {
              envSlide = releaseSlide;
              sustainTargetVol = releaseTargetVol;
              inRelease = true;
            } else {
              envSlide = 0;
            }
          } else {
            /* Starting volume already at or below sustain target (can happen
             * with low patVol where scaleVol rounds below the target).  If a
             * release envelope exists, start it immediately — the sustain
             * phase is effectively complete and waiting for a pKeyUp that may
             * never arrive would leave the note hanging at audible volume. */
            if (envSlide < 0 && runningVol > 0 && runningVol < stopAt
                && stopAt > 0 && !inRelease) {
              if (releaseSlide != 0) {
                int rs = releaseSlide;
                if (rs < -15) rs = -15;
                else if (rs > 15) rs = 15;
                envSlide = rs;
              } else {
                int s = (runningVol + xmTicksPerRow * 2 - 1) / (xmTicksPerRow * 2);
                if (s < 2) s = 2;
                if (s > 15) s = 15;
                envSlide = -s;
              }
              inRelease = true;
            } else {
              envSlide = 0;
            }
          }
        }

        /* Track pEnve volume changes in runningVol and block volume increases
         * on channels that have faded to silence.  In real TFMX, the pattern
         * envelope modifies CurVol directly; our Phase 2 must stay in sync
         * so the release/cleanup logic uses the correct baseline volume. */
        if (envSlide == 0 && cell->vol != 0) {
          unsigned char v = cell->vol;
          if (v >= 0x60 && v <= 0x6F) {
            runningVol -= (v & 0x0F) * xmTicksPerRow;
            if (runningVol < 0) runningVol = 0;
          } else if ((v >= 0x70 && v <= 0x7F) || (v > 0x10 && v <= 0x50)) {
            if (runningVol <= 0) {
              cell->vol = 0x10;
            } else if (v >= 0x70) {
              runningVol += (v & 0x0F) * xmTicksPerRow;
              if (runningVol > 64) runningVol = 64;
            } else {
              runningVol = v - 0x10;
            }
          }
        }

      }
    }
  }

  /* Place song loop (Bxx/Dxx) AFTER Phase 2 so panning/vibrato effects
   * don't overwrite the jump command.
   *
   * TFMX loops via an order-table jump (EFFE cmd 1).  In XM we encode
   * this as Bxx (jump to pattern) + Dxx (break to row within pattern).
   * Both commands are placed on the last row before the loop restarts,
   * in the first channel that has a free effect column.  If no channel
   * is free, channel 0 (or 1 for Dxx) is overwritten as a last resort. */
  if (loopJumpXmRow >= 0 && loopJumpTargetXmRow >= 0) {
    int srcRow = loopJumpXmRow;
    int dstPat = loopJumpTargetXmRow / ROWS_PER_PAT;
    int dstRow = loopJumpTargetXmRow % ROWS_PER_PAT;
    if (srcRow < MAX_TOTAL_ROWS) {
      bool placedB = false;
      for (int ch = 0; ch < numChans && !placedB; ch++) {
        XMCell* cell = &bigGrid[srcRow * numChans + ch];
        if (cell->fx == 0 && cell->param == 0) {
          cell->fx = 0x0B;
          cell->param = (unsigned char)dstPat;
          placedB = true;
        }
      }
      if (!placedB) {
        XMCell* cell = &bigGrid[srcRow * numChans + 0];
        cell->fx = 0x0B;
        cell->param = (unsigned char)dstPat;
      }

      if (dstRow > 0) {
        /* XM's Dxx uses BCD encoding: D32 = row 32, not D20 */
        unsigned char dstRowBCD = (unsigned char)(((dstRow / 10) << 4) | (dstRow % 10));
        bool placedD = false;
        for (int ch = 0; ch < numChans && !placedD; ch++) {
          XMCell* cell = &bigGrid[srcRow * numChans + ch];
          if (cell->fx == 0 && cell->param == 0) {
            cell->fx = 0x0D;
            cell->param = dstRowBCD;
            placedD = true;
          }
        }
        if (!placedD) {
          XMCell* cell = &bigGrid[srcRow * numChans + 1];
          cell->fx = 0x0D;
          cell->param = dstRowBCD;
        }
      }
    }
  }

  /* ===================================================================
   * PHASE 3: Write the XM file.
   * =================================================================== */

  /* Trim unused trailing channels to keep the XM file compact.
   * Overflow channels (attack transients) and stutter channels (pulsing
   * pads) are only used when specific macros require them.  We scan
   * backwards from the last channel and stop at the first one that
   * contains any data.  gridStride keeps the original allocation width
   * so bigGrid indexing stays valid; only xmChans (= written channels)
   * is reduced. */
  int gridStride = numChans;
  int xmChans = numChans;
  {
    int usedChans = BASE_CHANS;
    for (int ch = numChans - 1; ch >= BASE_CHANS; ch--) {
      bool hasData = false;
      for (int row = 0; row < totalXmRows && !hasData; row++) {
        XMCell* c = &bigGrid[row * gridStride + ch];
        if (c->note || c->inst || c->vol || c->fx || c->param)
          hasData = true;
      }
      if (hasData) { usedChans = ch + 1; break; }
    }
    xmChans = usedChans;
  }

  /* Slice total rows into 64-row XM patterns */
  int numPatterns = (totalXmRows + ROWS_PER_PAT - 1) / ROWS_PER_PAT;
  if (numPatterns < 1) numPatterns = 1;
  if (numPatterns > 256) numPatterns = 256;
  int songLen = numPatterns;

  /* detail info folded into summary line on stdout */

  /* XM header continuation (at file offset 60 + 4, inside the 276-byte block).
   * These 14 bytes define the global song properties.  restartPos tells the
   * XM player which pattern to jump to when reaching the end of the order
   * table — we set it to the pattern containing the TFMX loop target. */
  writeU16(out, (unsigned short)songLen);  /* song length (number of order entries) */
  int restartPos = (loopJumpTargetXmRow >= 0)
                   ? loopJumpTargetXmRow / ROWS_PER_PAT : 0;
  writeU16(out, (unsigned short)restartPos); /* restart position for looping */
  writeU16(out, (unsigned short)xmChans);  /* number of channels */
  writeU16(out, (unsigned short)numPatterns);
  writeU16(out, 128);                      /* number of instruments (always 128) */
  writeU16(out, 1);                        /* flags: bit 0 = use linear frequency table */
  writeU16(out, (unsigned short)xmSpeed);  /* default speed (ticks per row) */
  writeU16(out, (unsigned short)xmBPM);   /* default tempo (BPM) */

  /* Pattern order table: maps playback order → pattern index.
   * Since we generate patterns sequentially, order[i] = i.
   * XM requires exactly 256 bytes here (unused slots are zero). */
  unsigned char order[256];
  memset(order, 0, 256);
  for (int i = 0; i < numPatterns && i < 256; i++) order[i] = (unsigned char)i;
  fwrite(order, 1, 256, out);

  /* Write XM patterns with packed cell encoding.
   * XM supports two cell formats:
   *   0x80          → completely empty cell (1 byte, saves space)
   *   0x9F + 5 bytes → full cell: note, instrument, volume, effect, param
   * We always write the full 6-byte form for non-empty cells (0x9F = all
   * five fields present).  The bigGrid is sliced into ROWS_PER_PAT (64)
   * row chunks; each chunk becomes one XM pattern. */
  for (int pi = 0; pi < numPatterns; pi++) {
    int patStart = pi * ROWS_PER_PAT;
    int patEnd = patStart + ROWS_PER_PAT;
    if (patEnd > totalXmRows) patEnd = totalXmRows;
    int patRows = patEnd - patStart;
    if (patRows < 1) patRows = 1;

    unsigned char* packed = (unsigned char*)malloc((size_t)(patRows * xmChans * 6));
    if (!packed) { free(bigGrid); fclose(out); free(mdatBuf); free(smpl); if (logFile) fclose(logFile); return false; }
    unsigned char* p = packed;
    for (int row = patStart; row < patStart + patRows; row++) {
      for (int ch = 0; ch < xmChans; ch++) {
        XMCell* c = &bigGrid[row * gridStride + ch];
        if (c->note == 0 && c->inst == 0 && c->vol == 0 && c->fx == 0 && c->param == 0) {
          *p++ = 0x80;
        } else {
          *p++ = 0x9F;
          *p++ = c->note;
          *p++ = c->inst;
          *p++ = c->vol;
          *p++ = c->fx;
          *p++ = c->param;
        }
      }
    }
    size_t packedSize = (size_t)(p - packed);

    writeU32(out, 9);
    fputc(0, out);
    writeU16(out, (unsigned short)patRows);
    writeU16(out, (unsigned short)packedSize);
    fwrite(packed, 1, packedSize, out);
    free(packed);
  }
  free(bigGrid);

  /* ===================================================================
   * PHASE 4: Write 128 XM instruments.
   *
   * Slots 1-128 normally hold sustain instruments (one per TFMX macro).
   * For macros with mSetNote before mOn, the slot at (macroIdx + 65) is
   * repurposed as a one-shot attack instrument containing the pre-mOn
   * sample.  This attack instrument plays at a fixed pitch on an overflow
   * channel, matching the Amiga hardware's DMA behaviour.
   * =================================================================== */
  for (int inst = 0; inst < 128; inst++) {
    /* Check if this slot should hold an attack instrument instead of the
     * normal sustain instrument for macro[inst]. */
    int atkSrcMacro = -1;
    for (int m = 0; m < 64; m++) {
      if (atkInstMap[m] == inst + 1) { atkSrcMacro = m; break; }
    }

    int smpStart, smpLen;
    unsigned char loopType = 0;
    unsigned int xmLoopStart = 0, xmLoopLen = 0;
    signed char relNote = 0;
    char labelBuf[64];

    if (atkSrcMacro >= 0) {
      /* Attack instrument: one-shot, no loop, relativeNote = 0 */
      smpStart = macroSI[atkSrcMacro].atkStart;
      smpLen   = macroSI[atkSrcMacro].atkLen;
      if (smpStart + smpLen > (int)smplLen) { smpStart = 0; smpLen = 4; }
      if (smpLen < 4) smpLen = 4;
      loopType = 0;
      relNote  = 0;
      snprintf(labelBuf, sizeof(labelBuf), "Attack M%03d", atkSrcMacro);
    } else {
      MacroSampleInfo si = macroSI[inst];
      if (si.start + si.len > (int)smplLen) {
        si.start = 0; si.len = 512; si.oneShot = true;
      }
      if (si.len < 4) si.len = 4;

      smpStart = si.start;
      smpLen   = si.len;
      relNote  = (signed char)si.addNote;

      if (si.oneShot) {
        loopType = 0;
      } else if (si.hasSetLoop) {
        loopType = 1;
        xmLoopStart = (unsigned int)si.loopStart;
        xmLoopLen = (unsigned int)si.loopLen;
        if (xmLoopStart + xmLoopLen > (unsigned int)smpLen)
          xmLoopLen = (unsigned int)smpLen - xmLoopStart;
        if (xmLoopLen < 2) xmLoopLen = 2;
      } else {
        loopType = 1;
        xmLoopStart = 0;
        xmLoopLen = (unsigned int)smpLen;
      }
      if (inst == 0)
        snprintf(labelBuf, sizeof(labelBuf), "domy@noisebay.org");
      else
        snprintf(labelBuf, sizeof(labelBuf), "Instrument %03d", inst + 1);
    }

    signed char* bakedBuf = NULL;  /* non-NULL when sweep baking produces a new sample buffer */
    int bakedLen = 0;

    /* Song-patch attack prepend: some macros have a short attack transient
     * (different sample region) followed by the sustain body, but without
     * mSetNote.  The attack is prepended only when flagged by a song patch. */
    if (atkSrcMacro < 0 && atkPrepend[inst]) {
      int atkOff = atkPrependOff[inst];
      int atkSz  = atkPrependLen[inst];
      if (atkOff >= 0 && atkSz >= 4 && atkOff + atkSz <= (int)smplLen) {
        int totalLen = atkSz + smpLen;
        signed char* combined = (signed char*)malloc((size_t)totalLen);
        if (combined) {
          memcpy(combined, smpl + atkOff, (size_t)atkSz);
          memcpy(combined + atkSz, smpl + smpStart, (size_t)smpLen);
          bakedBuf = combined;
          bakedLen = totalLen;
          if (loopType != 0)
            xmLoopStart += (unsigned int)atkSz;
          smpLen = totalLen;
          if (logFile)
            fprintf(logFile, "  ATK PREPEND Inst %d: atkOff=0x%X atkSz=%d susOff=0x%X susSz=%d total=%d\n",
                   inst+1, atkOff, atkSz, smpStart, smpLen - atkSz, totalLen);
        }
      }
    }

    /* Positive mAddBegin sweep baking.
     *
     * On the Amiga, mAddBegin shifts the DMA read pointer forward through
     * the sample bank each frame, creating a timbral sweep (wavetable
     * synthesis).  XM has no per-frame sample-offset modulation, so we
     * "bake" the sweep by concatenating copies of the waveform at
     * progressively shifted offsets into a single longer sample.
     *
     * The baked sample consists of nLoops copies of the waveform (each
     * at a different offset), followed by one copy at the settled offset
     * which becomes the loop body.  RMS normalization ensures the baked
     * sample has the same perceived loudness as the original. */
    if (atkSrcMacro < 0 && !bakedBuf) {
      MacroSampleInfo& si2 = macroSI[inst];
      if (si2.sweepAmt > 0 && si2.sweepFrames > 0 && smpLen >= 4) {
        int origLen = smpLen;
        int hp = si2.sweepHalfPeriod;
        bool pingPong = !si2.oneShot && hp > 0;
        bool hasReverse = (si2.sweepAmt2 != 0 && si2.sweepReverseFrame > 0);

        int maxOffset;
        if (hasReverse)
          maxOffset = abs((si2.sweepReverseFrame - 1) * si2.sweepAmt);
        else if (pingPong)
          maxOffset = (hp + 1) * si2.sweepAmt;
        else
          maxOffset = si2.sweepAmt * si2.sweepFrames;

        int srcLen = origLen + maxOffset;
        if (smpStart + srcLen > (int)smplLen)
          srcLen = (int)smplLen - smpStart;
        if (srcLen > origLen) {
          const signed char* src = smpl + smpStart;
          int baseNote = 49;
          if (instFirstXmNote[inst] > 0)
            baseNote = instFirstXmNote[inst];
          int bytesPerFrame = (int)(256.0 * pow(2.0, (baseNote - 49 + relNote) / 12.0) + 0.5);
          if (bytesPerFrame < 32) bytesPerFrame = 32;

          int nFrames, nLoops;
          if (pingPong) {
            nFrames = 2 * hp;
          } else {
            nFrames = si2.sweepFrames;
            if (nFrames < 1) nFrames = 1;
          }
          nLoops = (nFrames * bytesPerFrame + origLen - 1) / origLen;
          if (nLoops < 2) nLoops = 2;
          if (nLoops > 2048) nLoops = 2048;

          int settledOffset = 0;
          if (hasReverse) {
            int peakOff = (si2.sweepReverseFrame - 1) * si2.sweepAmt;
            settledOffset = peakOff
                          + (si2.sweepFrames - si2.sweepReverseFrame + 1) * si2.sweepAmt2;
          } else if (!pingPong) {
            settledOffset = si2.sweepFrames * si2.sweepAmt;
          }
          if (settledOffset < 0) settledOffset = 0;
          if (settledOffset + origLen > srcLen) settledOffset = srcLen - origLen;
          if (settledOffset < 0) settledOffset = 0;

          int totalLen = nLoops * origLen + origLen;
          bakedLen = totalLen;
          bakedBuf = (signed char*)malloc((size_t)totalLen);
          if (bakedBuf) {
            int writePos = 0;
            for (int lp = 0; lp < nLoops; lp++) {
              int frame = (int)((double)lp * origLen / bytesPerFrame);
              if (frame >= nFrames) frame = nFrames - 1;
              int offset;
              if (pingPong) {
                int phase = frame % nFrames;
                if (phase < hp)
                  offset = (phase + 2) * si2.sweepAmt;
                else
                  offset = (nFrames - phase) * si2.sweepAmt;
              } else if (hasReverse && frame >= si2.sweepReverseFrame) {
                int peakOff = (si2.sweepReverseFrame - 1) * si2.sweepAmt;
                offset = peakOff + (frame - si2.sweepReverseFrame + 1) * si2.sweepAmt2;
              } else {
                offset = frame * si2.sweepAmt;
              }
              if (offset < 0) offset = 0;
              for (int b = 0; b < origLen; b++) {
                int readPos = offset + b;
                if (readPos >= 0 && readPos < srcLen)
                  bakedBuf[writePos++] = src[readPos];
                else
                  bakedBuf[writePos++] = 0;
              }
            }
            for (int b = 0; b < origLen; b++) {
              int readPos = settledOffset + b;
              if (readPos >= 0 && readPos < srcLen)
                bakedBuf[writePos++] = src[readPos];
              else
                bakedBuf[writePos++] = 0;
            }
            smpLen = totalLen;
            if (si2.oneShot) {
              loopType = 0;
              xmLoopStart = 0;
              xmLoopLen = 0;
            } else {
              loopType = 1;
              xmLoopStart = (unsigned int)(nLoops * origLen);
              xmLoopLen = (unsigned int)origLen;
            }

            /* Normalize baked amplitude to match original sample RMS.
             * Sweep baking reads beyond the original sample boundary,
             * which can raise the RMS significantly and distort the
             * volume balance between left/right instruments. */
            double origSumSq = 0;
            for (int k = 0; k < origLen; k++) {
              double v = src[k];
              origSumSq += v * v;
            }
            double origRms = sqrt(origSumSq / origLen);

            double bakedSumSq = 0;
            for (int k = 0; k < totalLen; k++) {
              double v = bakedBuf[k];
              bakedSumSq += v * v;
            }
            double bakedRms = sqrt(bakedSumSq / totalLen);

            if (logFile) {
              fprintf(logFile, "  SWEEP BAKE Inst %d: origLen=%d origRms=%.1f bakedLen=%d bakedRms=%.1f",
                     inst+1, origLen, origRms, totalLen, bakedRms);
            }
            if (bakedRms > origRms && bakedRms > 1.0) {
              double scale = origRms / bakedRms;
              if (logFile) fprintf(logFile, " -> NORM scale=%.3f\n", scale);
              for (int k = 0; k < totalLen; k++) {
                int v = (int)(bakedBuf[k] * scale);
                if (v > 127) v = 127;
                if (v < -128) v = -128;
                bakedBuf[k] = (signed char)v;
              }
            } else {
              if (logFile) fprintf(logFile, " -> SKIP (no norm needed)\n");
            }
          }
        }
      }
    }

    /* Negative mAddBegin sweep: not expressible in XM format.
     * On the Amiga, mAddBegin with negative amount shifts the DMA read
     * pointer backward through the SMPL bank each frame, reading bytes
     * from neighbouring instruments.  This creates the bass "wobble."
     * Baking this into the XM sample fails because the foreign sample
     * data (e.g. SFX) becomes audible as artifacts.  XM has no mechanism
     * for real-time sample-start modulation, so this effect is skipped. */

    /* mCont sweep attack prefix: when the source macro uses a tiny wavetable
     * with a positive sweep as its attack, and mCont redirects to a different
     * sustained sample, bake the sweep attack and prepend it to the sustained
     * sample.  The loop then covers only the sustained portion. */
    if (atkSrcMacro < 0 && !bakedBuf && macroContTarget[inst] >= 0) {
      MacroSampleInfo& srcSI = macroSrcSI[inst];
      if (srcSI.sweepAmt > 0 && srcSI.len > 0 && srcSI.len <= 128 && smpLen >= 4) {
        int atkOrigLen = srcSI.len;
        int atkStart = srcSI.start;
        int hp = srcSI.sweepHalfPeriod;
        int nFrames = (hp > 0) ? 2 * hp : srcSI.sweepFrames;
        if (nFrames < 1) nFrames = srcSI.sweepFrames;
        if (nFrames < 1) nFrames = 8;

        int maxOffset = (hp > 0) ? (hp + 1) * srcSI.sweepAmt : srcSI.sweepAmt * nFrames;
        int atkSrcLen = atkOrigLen + maxOffset;
        if (atkStart + atkSrcLen > (int)smplLen)
          atkSrcLen = (int)smplLen - atkStart;

        if (atkSrcLen > atkOrigLen) {
          int baseNote = 49;
          if (instFirstXmNote[inst] > 0) baseNote = instFirstXmNote[inst];
          int bytesPerFrame = (int)(256.0 * pow(2.0, (baseNote - 49 + relNote) / 12.0) + 0.5);
          if (bytesPerFrame < 32) bytesPerFrame = 32;

          int nLoops = (nFrames * bytesPerFrame + atkOrigLen - 1) / atkOrigLen;
          if (nLoops < 2) nLoops = 2;
          if (nLoops > 512) nLoops = 512;

          int atkBakedLen = nLoops * atkOrigLen;
          int totalLen = atkBakedLen + smpLen;
          bakedBuf = (signed char*)malloc((size_t)totalLen);
          if (bakedBuf) {
            const signed char* atkSrc = smpl + atkStart;
            int writePos = 0;
            for (int lp = 0; lp < nLoops; lp++) {
              int frame = (int)((double)lp * atkOrigLen / bytesPerFrame);
              if (frame >= nFrames) frame = nFrames - 1;
              int offset;
              if (hp > 0) {
                int phase = frame % (2 * hp);
                if (phase < hp)
                  offset = (phase + 2) * srcSI.sweepAmt;
                else
                  offset = (2 * hp - phase) * srcSI.sweepAmt;
              } else {
                offset = frame * srcSI.sweepAmt;
              }
              if (offset < 0) offset = 0;
              for (int b = 0; b < atkOrigLen; b++) {
                int readPos = offset + b;
                if (readPos >= 0 && readPos < atkSrcLen)
                  bakedBuf[writePos++] = atkSrc[readPos];
                else
                  bakedBuf[writePos++] = 0;
              }
            }
            /* Append the sustained sample */
            const signed char* susSrc = smpl + smpStart;
            for (int b = 0; b < smpLen; b++) {
              if (smpStart + b < (int)smplLen)
                bakedBuf[writePos++] = susSrc[b];
              else
                bakedBuf[writePos++] = 0;
            }
            bakedLen = totalLen;
            loopType = 1;
            xmLoopStart = (unsigned int)atkBakedLen + xmLoopStart;
            xmLoopLen = (xmLoopLen > 0) ? xmLoopLen : (unsigned int)smpLen;
            if (xmLoopStart + xmLoopLen > (unsigned int)totalLen)
              xmLoopLen = (unsigned int)totalLen - xmLoopStart;
            smpLen = totalLen;

            if (logFile)
              fprintf(logFile, "  SWEEP ATK PREFIX Inst %d: atkLen=%d sweepFrames=%d atkBaked=%d sustainLen=%d total=%d\n",
                     inst+1, atkOrigLen, nFrames, atkBakedLen, smpLen - atkBakedLen, totalLen);
          }
        }
      }
    }

    /* XM instrument header: 263 = 29 (base) + 234 (extra when numSamples>0).
     * We always emit exactly one sample per instrument. */
    writeU32(out, 263);

    char instName[22];
    memset(instName, 0, 22);
    { size_t n = strlen(labelBuf); if (n > 22) n = 22; memcpy(instName, labelBuf, n); }
    fwrite(instName, 1, 22, out);
    fputc(0, out);          /* instrument type (always 0) */
    writeU16(out, 1);       /* number of samples in this instrument */

    /* Extra instrument header (present because numSamples = 1).
     * We don't use XM's built-in volume/panning envelopes — all envelope
     * behaviour is baked into the pattern data by Phase 2.  So these
     * fields are zeroed out (disabled). */
    writeU32(out, 40);      /* sample header size */
    for (int i = 0; i < 96; i++) fputc(0, out);  /* note-to-sample map (all → sample 0) */
    for (int i = 0; i < 12; i++) { writeU16(out, 0); writeU16(out, 0); }  /* volume envelope points (unused) */
    for (int i = 0; i < 12; i++) { writeU16(out, 0); writeU16(out, 32); } /* panning envelope points (unused) */
    fputc(0, out); fputc(0, out);    /* num volume / panning envelope points */
    fputc(0, out); fputc(0, out); fputc(0, out);  /* vol sustain, loop start, loop end */
    fputc(0, out); fputc(0, out); fputc(0, out);  /* pan sustain, loop start, loop end */
    fputc(0, out); fputc(0, out);    /* volume / panning envelope type (0 = disabled) */
    fputc(0, out); fputc(0, out); fputc(0, out); fputc(0, out);  /* vibrato type/sweep/depth/rate */
    writeU16(out, 256);     /* volume fadeout (fast, since we manage fades ourselves) */
    { unsigned char res[22]; memset(res, 0, 22); fwrite(res, 1, 22, out); } /* reserved */

    /* Sample header (40 bytes).
     * Volume is always 64 (max) — actual dynamics are in the pattern data.
     * relativeNote offsets the playback pitch by mAddNote semitones so
     * XM note C-4 produces the same frequency as the original TFMX note. */
    writeU32(out, (unsigned int)smpLen);       /* sample length in bytes */
    writeU32(out, xmLoopStart);               /* loop start offset */
    writeU32(out, xmLoopLen);                 /* loop length (0 if no loop) */
    fputc(64, out);                           /* default volume (always max) */
    fputc(0, out);                            /* finetune (signed, 0 = none) */
    fputc(loopType, out);                     /* 0=none, 1=forward, 2=ping-pong */
    fputc(instSamplePan[inst], out);           /* panning (baked from channel usage) */
    fputc((unsigned char)relNote, out);       /* relative note (from mAddNote) */
    fputc(0, out);                            /* encoding: 0 = delta-encoded 8-bit */
    memset(instName, 0, 22);
    { size_t n = strlen(labelBuf); if (n > 22) n = 22; memcpy(instName, labelBuf, n); }
    fwrite(instName, 1, 22, out);

    /* Diagnostic: compute and log sample RMS / Peak */
    {
      const signed char* smpData = bakedBuf ? bakedBuf : (const signed char*)(smpl + smpStart);
      int smpDiagLen = smpLen;
      if (smpDiagLen > 0 && logFile) {
        double sumSq = 0;
        int peak = 0;
        for (int k = 0; k < smpDiagLen; k++) {
          int v = smpData[k];
          sumSq += (double)v * v;
          if (abs(v) > peak) peak = abs(v);
        }
        double rms = sqrt(sumSq / smpDiagLen);
        fprintf(logFile, "  SMP Inst %3d: len=%6d  RMS=%5.1f  Peak=%4d  pan=0x%02X  %s\n",
               inst + 1, smpDiagLen, rms, peak, instSamplePan[inst], labelBuf);
      }
    }

    /* Write delta-encoded sample data.
     * XM stores 8-bit samples as deltas: byte[n] = sample[n] - sample[n-1].
     * If sweep baking produced a bakedBuf, use that; otherwise read
     * directly from the original TFMX smpl bank. */
    if (bakedBuf) {
      unsigned char* delta = (unsigned char*)malloc((size_t)smpLen);
      if (delta) {
        deltaEncode8(bakedBuf, smpLen, delta);
        fwrite(delta, 1, (size_t)smpLen, out);
        free(delta);
      }
      free(bakedBuf);
    } else {
      unsigned char* delta = (unsigned char*)malloc((size_t)smpLen);
      if (delta) {
        deltaEncode8(smpl + smpStart, smpLen, delta);
        fwrite(delta, 1, (size_t)smpLen, out);
        free(delta);
      }
    }
  }  /* end of instrument write */

  fclose(out);
  free(mdatBuf);
  free(smpl);
  printf("Converted: %s  (%d patterns, %d channels, %d rows)  Errors: %d\n",
         outPath, numPatterns, xmChans, totalXmRows, xmErrors);
  if (logFile) {
    fprintf(logFile, "\nConverted: %s  (%d patterns, %d channels, %d rows)  Errors: %d\n",
           outPath, numPatterns, xmChans, totalXmRows, xmErrors);
    fclose(logFile);
    printf("  More info in: %s\n", logPath);
  }
  return true;
}
