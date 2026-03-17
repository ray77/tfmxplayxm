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
#ifndef _WIN32
#include <arpa/inet.h>
#endif

#include "tfmx.h"

#ifndef TFMXPLAY_VERSION
#define TFMXPLAY_VERSION "0.0.2"
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
 * Scale a linear TFMX volume (0-64) for XM output using a sqrt curve.
 *
 * The TFMX player applies a 2x amplitude boost and crossfeed in its audio
 * output path that XM trackers don't replicate.  Additionally, with 12 XM
 * channels vs 4 Amiga channels, each channel contributes proportionally less
 * to the mix.  A sqrt curve compensates by boosting quieter volumes more than
 * loud ones, preserving the perceived dynamic balance — especially important
 * for pEnve fades where target volumes like 12/64 would otherwise sound far
 * too quiet relative to attack levels.
 *
 * Mapping examples:  0→0, 12→28, 35→47, 48→55, 64→64
 */
static int scaleVol(int v) {
  if (v <= 0) return 0;
  if (v >= 64) return 64;
  double scaled = sqrt((double)v / 64.0) * 64.0;
  int result = (int)(scaled + 0.5);
  if (result > 64) result = 64;
  if (result < 1) result = 1;
  return result;
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
  int sustainSlide;   /* volume slide per row during sustain; negative = down */
  int sustainDelay;   /* rows to wait before sustain slide begins (from mWait) */
  int releaseSlide;   /* volume slide per row after pKeyUp; negative = down */
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
  MacroSampleInfo info = { 0, 512, 0, 0, false, false, 0, -1, 0, -1 };
  int curStart = 0;
  int curLenWords = 256;   /* TFMX lengths are in 16-bit words */
  bool gotAny = false;
  bool gotAddNote = false;
  bool afterOn = false;
  int preOnSetNote = -1;   /* mSetNote value found before mOn */

  int onStart = 0, onLen = 512;
  int postStart = -1, postLen = -1;

  for (int i = 0; i < 256; i++) {
    switch (macro[i].op) {
      case mSetBegin:
        curStart = (macro[i].data[0]<<16)|(macro[i].data[1]<<8)|(macro[i].data[2]);
        if (afterOn && postStart < 0) postStart = curStart;
        else if (afterOn) postStart = curStart;
        break;
      case mSetLen:
        curLenWords = (macro[i].data[1]<<8)|(macro[i].data[2]);
        if (curLenWords <= 0) curLenWords = 256;
        if (afterOn && postLen < 0) postLen = curLenWords;
        else if (afterOn) postLen = curLenWords;
        break;
      case mSetNote:
        if (!afterOn) preOnSetNote = macro[i].data[0];
        break;
      case mOn:
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
      case mAddNote:
        if (!gotAddNote) {
          info.addNote = (signed char)macro[i].data[0];
          gotAddNote = true;
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
       * attack info so the simulation can emit it on an overflow channel. */
      if (preOnSetNote >= 0) {
        info.atkStart   = onStart;
        info.atkLen     = onLen * 2;
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
  return info;
}

/*
 * Convert a TFMX per-tick envelope rate to an XM per-row volume slide speed.
 *
 * TFMX applies envAmt every (envTimeC+1) ticks. XM applies the volume slide
 * once per row, which lasts (xmSpeed-1) ticks (first tick is the trigger).
 * We scale so the total volume change over the same wall-clock time matches.
 * Result is clamped to XM's 1-15 range.
 */
static int calcEnvSpeed(int envAmt, int envTimeC, int xmSpeed) {
  int denom = (envTimeC + 1) * (xmSpeed > 1 ? (xmSpeed - 1) : 1);
  int speed = (envAmt * xmSpeed + denom - 1) / denom;
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
 * For multi-phase macros, the LAST mAddVol/mSetVol wins, since commands after
 * mOn override the initial attack phase and represent the sustained sound.
 */
static MacroEffectInfo getMacroEffects(const TFMXMacroData macro[256], int xmSpeed) {
  MacroEffectInfo fx = {-999, -1, 0, 0, 0, 0, 0};
  bool foundVib = false;
  bool afterOn = false;
  bool afterWaitUp = false;
  bool foundSustainEnv = false;
  int waitBeforeFirstEnv = 0;

  for (int i = 0; i < 256; i++) {
    switch (macro[i].op) {
      case mAddVol:
        /* For multi-phase macros, use the LAST addVol (sustain phase) */
        fx.addVol = (signed char)macro[i].data[2];  /* SIGNED! 252 → -4 */
        fx.setVol = -1; /* addVol overrides setVol */
        break;
      case mSetVol:
        fx.setVol = macro[i].data[2];
        fx.addVol = -999; /* setVol overrides addVol */
        break;
      case mOn:
        afterOn = true;
        break;
      case mWait:
        if (afterOn && !foundSustainEnv) {
          int w = (macro[i].data[0]<<16)|(macro[i].data[1]<<8)|(macro[i].data[2]);
          waitBeforeFirstEnv += w + 1;
        }
        break;
      case mWaitUp:
        afterWaitUp = true;
        break;
      case mWaitSample:
        break;
      case mVibrato: {
        /* TFMX vibrato: data[0]=period (ticks per half-cycle, bit0 ignored),
         * data[2]=amplitude. XM effect 4xy: x=speed (higher=faster),
         * y=depth. We approximate the rate and depth mapping. */
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
          }
          foundVib = true;
        }
        break;
      }
      case mEnv: {
        /* mEnv: data[0]=amount, data[1]=time counter, data[2]=target volume.
         * TFMX adjusts volume by 'amount' every (timeC+1) ticks toward target.
         *
         * Before mWaitUp: this is the sustain envelope (applied while note
         * is held — e.g., a slow fade from full volume to a sustain level).
         * After mWaitUp: this is the release envelope (applied after pKeyUp
         * — typically a fast fade to 0). */
        int envAmt = macro[i].data[0];
        int envTimeC = macro[i].data[1];
        int envTarget = macro[i].data[2];
        int speed = calcEnvSpeed(envAmt, envTimeC, xmSpeed);

        if (!foundSustainEnv && !afterWaitUp) {
          fx.sustainDelay = (waitBeforeFirstEnv + xmSpeed - 1) / xmSpeed;
          if (fx.sustainDelay > 16) fx.sustainDelay = 16;
          if (envTarget == 0) {
            fx.sustainSlide = -speed;
          } else {
            int initVol = (fx.addVol != -999) ? fx.addVol + 24 :
                          (fx.setVol >= 0) ? fx.setVol : 48;
            fx.sustainSlide = (envTarget > initVol) ? speed : -speed;
          }
          foundSustainEnv = true;
        } else if (afterWaitUp && envTarget == 0) {
          fx.releaseSlide = -speed;
        }
        break;
      }
      case mStop: goto efxdone;
      default: break;
    }
  }
efxdone:
  /* If the macro has a sustain fade but no explicit release envelope,
   * reuse the sustain slide as the release — otherwise notes hang forever. */
  if (fx.releaseSlide == 0 && foundSustainEnv && fx.sustainSlide < 0) {
    fx.releaseSlide = fx.sustainSlide;
  }
  return fx;
}

/*
 * Delta-encode 8-bit sample data for the XM format.
 * XM stores samples as deltas: each byte = (current - previous) & 0xFF.
 * This reduces entropy for compression and is required by the spec.
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
bool convertToXM(const char* mdatPath, const char* smplPath, const char* outPath, int subsong) {
  printf("XM-Export: multi-pattern, instrument 263 -> %s\n", outPath);
  FILE* f;
  TFMXHeader head;
  int patPoint[128], macroPoint[128];
  TFMXOrders track[128][8];
  TFMXMacroData macro[128][256];
  signed char* smpl = NULL;
  size_t smplLen = 0;

  /* Load smpl */
  f = fopen(smplPath, "rb");
  if (!f) {
    perror(smplPath);
    return false;
  }
  fseek(f, 0, SEEK_END);
  smplLen = ftell(f);
  fseek(f, 0, SEEK_SET);
  smpl = (signed char*)malloc(smplLen);
  if (!smpl) {
    fclose(f);
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
    return false;
  }
  fseek(f, 0, SEEK_END);
  size_t mdatSize = ftell(f);
  fseek(f, 0, SEEK_SET);
  unsigned char* mdatBuf = (unsigned char*)malloc(mdatSize);
  if (!mdatBuf) {
    fclose(f);
    free(smpl);
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
    if (macroPoint[i] == 0) {
      macro[i][0].op = mStop;
      continue;
    }
    int off = macroPoint[i];
    int s = 0;
    while (s < 256 && off + 4 <= (int)mdatSize) {
      macro[i][s].op = mdatBuf[off];
      macro[i][s].data[0] = mdatBuf[off + 1];
      macro[i][s].data[1] = mdatBuf[off + 2];
      macro[i][s].data[2] = mdatBuf[off + 3];
      if (macro[i][s].op == mStop) break;
      s++;
      off += 4;
    }
  }

  /* Open output XM */
  FILE* out = fopen(outPath, "wb");
  if (!out) {
    perror(outPath);
    free(mdatBuf);
    free(smpl);
    return false;
  }

  /* XM pre-header: "Extended Module: " (17) + name 20 + 0x1A + tracker 20 + version 2 + header_size 4 */
  fwrite("Extended Module: ", 1, 17, out);
  char xmName[21];
  memset(xmName, 0, sizeof(xmName));
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
  int numChans = BASE_CHANS + OVERFLOW_CHANS;

  /* One cell per channel per row — matches XM's packed pattern cell format */
  struct XMCell { unsigned char note, inst, vol, fx, param; };
  XMCell* bigGrid = (XMCell*)calloc((size_t)MAX_TOTAL_ROWS * numChans, sizeof(XMCell));
  if (!bigGrid) { fclose(out); free(mdatBuf); free(smpl); return false; }

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
  bool chanAlive[8];
  int lastEnveVol[8]; /* last scaled volume written by pEnve, for computing slides */
  for (int c = 0; c < 8; c++) {
    chanEnv[c].active = false;
    chanAlive[c] = false;
    lastEnveVol[c] = 0;
  }

  int totalXmRows = 0;
  int ordRowToXmRow[128];
  memset(ordRowToXmRow, -1, sizeof(ordRowToXmRow));
  int loopJumpXmRow = -1;
  int loopJumpTargetXmRow = -1;

  /* TFMX speed = ticks per row. The header stores (speed - 1). */
  int xmSpeed = head.songSpeed[subsong] + 1;
  if (xmSpeed < 1) xmSpeed = 6;
  if (xmSpeed > 31) xmSpeed = 31;

  /* Pre-compute macro effects for all 128 instruments.
   * Needed before simulation so pEnve can initialize chanEnv.vol to the
   * correct effective volume (accounting for mAddVol/mSetVol). */
  MacroEffectInfo macroFX[128];
  for (int i = 0; i < 128; i++) macroFX[i] = getMacroEffects(macro[i], xmSpeed);

  /* Pre-compute sample info for all macros (needed for attack overflow).
   * For macros with mSetNote before mOn, the pre-mOn sample is a fixed-pitch
   * attack transient.  We map each such macro to an attack XM instrument
   * stored in the upper instrument slots (macro + 65). */
  MacroSampleInfo macroSI[128];
  int atkInstMap[128];  /* atkInstMap[m] = XM instrument number (1-based) for attack; 0 = none */
  for (int i = 0; i < 128; i++) {
    macroSI[i] = getMacroSample(macro[i]);
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

  /* Track which stereo side each instrument plays on so we can bake the
   * panning into the sample header, freeing the effect column for E9x etc. */
  int instPlayLeft[128];
  int instPlayRight[128];
  memset(instPlayLeft, 0, sizeof(instPlayLeft));
  memset(instPlayRight, 0, sizeof(instPlayRight));

  int startRow = head.songStart[subsong];
  int endRow = head.songEnd[subsong];
  if (startRow < 0) startRow = 0;
  if (endRow < startRow) endRow = startRow;

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
      ts[t].trans = (signed char)track[ordRow][t].trans;
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
      int cmd = track[ordRow][1].trans;
      if (cmd == 0) break;
      if (cmd == 1) {
        int jumpTarget = track[ordRow][2].trans;
        if (jumpTarget <= ordRow) {
          loopJumpXmRow = totalXmRows > 0 ? totalXmRows - 1 : 0;
          loopJumpTargetXmRow = ordRowToXmRow[jumpTarget];
          break;
        }
        ordRow = jumpTarget - 1;
      }
      if (cmd == 2) {
        /* TODO: tempo changes not yet mapped to XM Fxx effect */
      }
      continue;
    }

    ordRowToXmRow[ordRow] = totalXmRows;
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
              /* Pause this track for item.ins ticks before processing next command */
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
              /* Pattern-level volume envelope: ramp the target channel's volume
               * from its current level toward 'detune' by 'ins' units every
               * (vol+1) ticks. Used for chord pad fades and other gradual changes. */
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
              /* Release the note on the target channel.
               * Mapped to XM note-off (97), which triggers the release phase
               * of the macro envelope in post-processing. */
              int ch = item.chan;
              if (ch >= 0 && ch < BASE_CHANS && totalXmRows < MAX_TOTAL_ROWS)
                bigGrid[totalXmRows * numChans + ch].note = 97;
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
              if ((item.note & 0xc0) == 0xc0) break; /* portamento — not yet mapped */

              {
                /* Regular note event. TFMX note = 6-bit pitch + transposition.
                 * The +25 offset maps TFMX notes to XM so that the effective
                 * playback frequency (after per-instrument relativeNote from
                 * mAddNote) matches the original TFMX NTSC period within 0.2%. */
                int rawNote = (item.note & 0x3f) + ts[t].trans;
                int xmNote = rawNote + 25;
                if (xmNote < 1) xmNote = 1;
                if (xmNote > 96) xmNote = 96;
                int ch = item.chan;
                if (ch < 0 || ch >= BASE_CHANS) ch = 0;
                if (totalXmRows < MAX_TOTAL_ROWS) {
                  XMCell* cell = &bigGrid[totalXmRows * numChans + ch];
                  cell->note = (unsigned char)xmNote;
                  cell->inst = (unsigned char)(item.ins + 1);
                  /* Preliminary volume from pattern data (patVol * 4).
                   * This will be corrected to effective volume in post-processing. */
                  int v = item.vol;
                  cell->vol = (unsigned char)(0x10 + v * 4);
                  if (cell->vol > 0x50) cell->vol = 0x50;
                  /* Initialize the pEnve envelope volume to the effective
                   * TFMX volume (accounting for macro mAddVol/mSetVol),
                   * so pattern envelopes fade from the correct starting level. */
                  int idx = item.ins;
                  if (idx >= 0 && idx < 128) {
                    MacroEffectInfo& ef = macroFX[idx];
                    int effVol;
                    if (ef.setVol >= 0)        effVol = ef.setVol;
                    else if (ef.addVol != -999) effVol = ef.addVol + v * 3;
                    else                        effVol = v * 3;
                    if (effVol > 64) effVol = 64;
                    if (effVol < 0) effVol = 0;
                    chanEnv[ch].vol = effVol;
                  } else {
                    chanEnv[ch].vol = v * 4;
                  }
                  chanEnv[ch].active = false;
                  chanAlive[ch] = true;
                  lastEnveVol[ch] = scaleVol(chanEnv[ch].vol);

                  /* Track stereo side for sample-header panning */
                  if (item.ins >= 0 && item.ins < 128) {
                    bool isR = (ch & 1) ^ ((ch & 2) >> 1);
                    if (isR) instPlayRight[item.ins]++;
                    else     instPlayLeft[item.ins]++;
                  }

                  /* Multi-phase attack: mSetNote-before-mOn → one-shot on overflow.
                   * Only four overflow columns (XM 9–12): Amiga ch 0–3.  Ch 4–7 keep
                   * sustain-only on the main column (no extra XM channels). */
                  int macIdx = item.ins;
                  if (macIdx >= 0 && macIdx < 128 && atkInstMap[macIdx] > 0 &&
                      ch >= 0 && ch < OVERFLOW_CHANS) {
                    int ovfCh = BASE_CHANS + ch;
                    int atkNote = macroSI[macIdx].setNoteVal + 25;
                    if (atkNote >= 1 && atkNote <= 96) {
                      XMCell* atkCell = &bigGrid[totalXmRows * numChans + ovfCh];
                      atkCell->note = (unsigned char)atkNote;
                      atkCell->inst = (unsigned char)atkInstMap[macIdx];
                      atkCell->vol  = cell->vol;
                      int atkIdx = atkInstMap[macIdx] - 1;
                      if (atkIdx >= 0 && atkIdx < 128) {
                        bool isR = (ch & 1) ^ ((ch & 2) >> 1);
                        if (isR) instPlayRight[atkIdx]++;
                        else     instPlayLeft[atkIdx]++;
                      }
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
          /* ~¾ pattern-tick rate: full xmSpeed made pEnve hits reach target too fast */
          int envFrames = (xmSpeed * 2 + 2) / 3; /* ~⅔ tick — etwas langsamer als ¾ */
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
                if (chanEnv[c].target == 0)
                  chanAlive[c] = false;
              }
            }
          }
          /* Only write pEnve volumes if a note was placed on this channel
           * recently. In TFMX every pEnve cycle is preceded by a note, but
           * some notes are lost in conversion (e.g. when a track redirects
           * via pPPat and the note timing doesn't align with our XM grid).
           * Without this check, the pEnve would raise the volume of an old,
           * decayed note — producing audible "ghost" tones. */
          {
            bool hasRecentNote = false;
            int lookback = 32;
            for (int back = 0; back <= lookback && (totalXmRows - back) >= 0; back++) {
              XMCell* prev = &bigGrid[(totalXmRows - back) * numChans + c];
              if (prev->note > 0 && prev->note < 97 && prev->inst > 0) {
                hasRecentNote = true;
                break;
              }
            }
            if (!hasRecentNote) continue;
          }
          XMCell* cell = &bigGrid[totalXmRows * numChans + c];
          if (cell->note == 0 && cell->inst == 0 && cell->vol == 0) {
            int v = chanEnv[c].vol;
            if (v < 0) v = 0;
            if (v > 64) v = 64;
            int newScaled = scaleVol(v);
            int delta = lastEnveVol[c] - newScaled;
            int slideTicks = (xmSpeed > 1) ? (xmSpeed - 1) : 1;
            if (delta > 0) {
              int speed = (delta + slideTicks - 1) / slideTicks;
              const int minRows = 7;
              if (delta > slideTicks * 2 && speed > 1) {
                int cap = (delta + minRows * slideTicks - 1) / (minRows * slideTicks);
                if (cap >= 1 && cap < speed) speed = cap;
              }
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
            /* delta == 0: no change needed, leave cell empty */
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
  unsigned char instSamplePan[128];
  bool instNeedsPanFx[128];
  for (int i = 0; i < 128; i++) {
    if (instPlayRight[i] > 0 && instPlayLeft[i] == 0) {
      instSamplePan[i] = 0xD0;
      instNeedsPanFx[i] = false;
    } else if (instPlayLeft[i] > 0 && instPlayRight[i] == 0) {
      instSamplePan[i] = 0x30;
      instNeedsPanFx[i] = false;
    } else if (instPlayLeft[i] > 0 && instPlayRight[i] > 0) {
      instSamplePan[i] = 128;
      instNeedsPanFx[i] = true;
    } else {
      instSamplePan[i] = 128;
      instNeedsPanFx[i] = false;
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
    bool vibActive = false;
    bool vibPending = false;
    int envSlide = 0;
    int envDelayLeft = 0;
    int releaseSlide = 0;
    bool inRelease = false;
    int curVibS = 0, curVibD = 0;
    int runningVol = 0; /* track actual volume to stop slides at 0 */
    bool isRight = (ch & 1) ^ ((ch & 2) >> 1);
    unsigned char panParam = isRight ? 0xD0 : 0x30;

    for (int row = 0; row < totalXmRows; row++) {
      XMCell* cell = &bigGrid[row * numChans + ch];

      if (cell->note > 0 && cell->note < 97 && cell->inst > 0) {
        /* --- New note: compute effective volume and set up effects --- */
        int idx = cell->inst - 1;
        if (idx >= 0 && idx < 128) {
          MacroEffectInfo& ef = macroFX[idx];

          /* Recover the original TFMX patVol (0-15) from the preliminary
           * volume we stored during simulation, then compute the effective
           * volume using the macro's volume modifiers. */
          int rawPV = ((int)cell->vol - 0x10);
          if (rawPV < 0) rawPV = 0;
          int patVol = rawPV / 4;
          int effVol;
          if (ef.setVol >= 0)        effVol = ef.setVol;
          else if (ef.addVol != -999) effVol = ef.addVol + patVol * 3;
          else                        effVol = patVol * 3;
          if (effVol > 64) effVol = 64;
          if (effVol < 0) effVol = 0;
          int scaledEV = scaleVol(effVol);
          cell->vol = (unsigned char)(0x10 + scaledEV);
          runningVol = scaledEV;

          /* Panning: only needed for instruments that play on both stereo
           * sides.  For single-side instruments, the sample header provides
           * the correct panning on note trigger automatically. */
          if (instNeedsPanFx[idx]) {
            cell->fx = 8;
            cell->param = panParam;
          }

          /* Vibrato: place directly on the note row when the effect column
           * is free.  If panning already occupies it, defer to the next row. */
          if (ef.vibSpeed > 0 && ef.vibDepth > 0) {
            curVibS = ef.vibSpeed; curVibD = ef.vibDepth;
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

          /* Set up sustain and release envelope state for this note.
           * Scale slide speed proportionally so release duration stays the
           * same despite the boosted initial volume. */
          envSlide = ef.sustainSlide;
          if (envSlide != 0 && effVol > 0) {
            /* Slower slides (~½): fades were ending audibly too soon vs TFMX. */
            int s = (abs(envSlide) * scaledEV * 2 + effVol * 5 - 1) / (effVol * 5);
            if (s < 1) s = 1;
            if (s > 15) s = 15;
            envSlide = (envSlide < 0) ? -s : s;
          }
          envDelayLeft = ef.sustainDelay;
          releaseSlide = ef.releaseSlide;
          if (releaseSlide != 0 && effVol > 0) {
            int s = (abs(releaseSlide) * scaledEV * 2 + effVol * 5 - 1) / (effVol * 5);
            if (s < 1) s = 1;
            if (s > 15) s = 15;
            releaseSlide = (releaseSlide < 0) ? -s : s;
          }
          inRelease = false;
        }
      } else if (cell->note == 97) {
        /* --- Key-up event (from pKeyUp): switch to release envelope ---
         * We clear the note-off (setting note=0) because XM's note-off
         * without a volume envelope would immediately silence the channel.
         * Instead, we apply a gradual volume slide (release envelope). */
        cell->note = 0;
        if (releaseSlide != 0) {
          envSlide = releaseSlide;
          envDelayLeft = 0;
          inRelease = true;
        } else if (envSlide < 0) {
          inRelease = true;
        } else {
          envSlide = -2; /* fallback: gentle fade if no release defined */
          inRelease = true;
        }
        vibActive = false;
        vibPending = false;
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
        /* Volume envelope slide: track running volume and stop at 0.
         * Without this, slides continue endlessly across section boundaries
         * where no new notes appear to reset the envelope state. */
        if (envSlide != 0 && cell->vol == 0) {
          if (envDelayLeft > 0) {
            envDelayLeft--;
          } else if (runningVol > 0) {
            runningVol += envSlide;
            if (runningVol <= 0) {
              runningVol = 0;
              envSlide = 0;
              vibActive = false;
              vibPending = false;
            } else {
              if (envSlide < 0)
                cell->vol = (unsigned char)(0x60 + (-envSlide));
              else if (envSlide > 0)
                cell->vol = (unsigned char)(0x70 + envSlide);
            }
          } else {
            envSlide = 0;
          }
        }
      }
    }
  }

  /* Place song loop (Bxx/Dxx) AFTER Phase 2 so panning/vibrato effects
   * don't overwrite the jump command. */
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

  /* Slice total rows into 64-row XM patterns */
  int numPatterns = (totalXmRows + ROWS_PER_PAT - 1) / ROWS_PER_PAT;
  if (numPatterns < 1) numPatterns = 1;
  if (numPatterns > 256) numPatterns = 256;
  int songLen = numPatterns;

  printf("XM-Export: %d total rows -> %d patterns (%d rows/pat), speed=%d BPM=125\n",
         totalXmRows, numPatterns, ROWS_PER_PAT, xmSpeed);

  /* XM header fields (at offset 60, inside the 276-byte header block) */
  writeU16(out, (unsigned short)songLen);  /* song length in patterns */
  int restartPos = (loopJumpTargetXmRow >= 0)
                   ? loopJumpTargetXmRow / ROWS_PER_PAT : 0;
  writeU16(out, (unsigned short)restartPos); /* restart position */
  writeU16(out, (unsigned short)numChans);  /* number of channels */
  writeU16(out, (unsigned short)numPatterns);
  writeU16(out, 128);                      /* number of instruments */
  writeU16(out, 1);                        /* flags: 1 = linear frequency table */
  writeU16(out, (unsigned short)xmSpeed);  /* default speed (ticks/row) */
  writeU16(out, 125);                      /* default BPM */

  /* Order table: sequential pattern indices (0, 1, 2, ...) padded to 256 bytes */
  unsigned char order[256];
  memset(order, 0, 256);
  for (int i = 0; i < numPatterns && i < 256; i++) order[i] = (unsigned char)i;
  fwrite(order, 1, 256, out);

  /* Write XM patterns with packed cell encoding.
   * Each cell is either 0x80 (completely empty) or 0x9F followed by all 5
   * fields (note, instrument, volume, effect type, effect parameter). */
  for (int pi = 0; pi < numPatterns; pi++) {
    int patStart = pi * ROWS_PER_PAT;
    int patEnd = patStart + ROWS_PER_PAT;
    if (patEnd > totalXmRows) patEnd = totalXmRows;
    int patRows = patEnd - patStart;
    if (patRows < 1) patRows = 1;

    unsigned char* packed = (unsigned char*)malloc((size_t)(patRows * numChans * 6));
    if (!packed) { free(bigGrid); fclose(out); free(mdatBuf); free(smpl); return false; }
    unsigned char* p = packed;
    for (int row = patStart; row < patStart + patRows; row++) {
      for (int ch = 0; ch < numChans; ch++) {
        XMCell* c = &bigGrid[row * numChans + ch];
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
    char labelBuf[22];

    if (atkSrcMacro >= 0) {
      /* Attack instrument: one-shot, no loop, relativeNote = 0 */
      smpStart = macroSI[atkSrcMacro].atkStart;
      smpLen   = macroSI[atkSrcMacro].atkLen;
      if (smpStart + smpLen > (int)smplLen) { smpStart = 0; smpLen = 4; }
      if (smpLen < 4) smpLen = 4;
      loopType = 0;
      relNote  = 0;
      snprintf(labelBuf, 22, "Atk M%03d", atkSrcMacro);
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
      snprintf(labelBuf, 22, "Instrument %03d", inst + 1);
    }

    writeU32(out, 263);  /* instrument header size (including sample header) */

    char instName[22];
    memset(instName, 0, 22);
    snprintf(instName, 22, "%s", labelBuf);
    fwrite(instName, 1, 22, out);
    fputc(0, out);          /* instrument type (always 0) */
    writeU16(out, 1);       /* number of samples in this instrument */

    /* Extra header (only present when numSamples > 0) */
    writeU32(out, 40);      /* sample header size */
    for (int i = 0; i < 96; i++) fputc(0, out);  /* sample-for-note table (all map to sample 0) */
    for (int i = 0; i < 12; i++) { writeU16(out, 0); writeU16(out, 0); }  /* volume envelope points */
    for (int i = 0; i < 12; i++) { writeU16(out, 0); writeU16(out, 32); } /* panning envelope points */
    fputc(0, out); fputc(0, out);    /* num volume / panning envelope points */
    fputc(0, out); fputc(0, out); fputc(0, out);  /* vol sustain, loop start, loop end */
    fputc(0, out); fputc(0, out); fputc(0, out);  /* pan sustain, loop start, loop end */
    fputc(0, out); fputc(0, out);    /* volume / panning envelope type (0 = disabled) */
    fputc(0, out); fputc(0, out); fputc(0, out); fputc(0, out);  /* vibrato type/sweep/depth/rate */
    writeU16(out, 256);     /* volume fadeout */
    { unsigned char res[22]; memset(res, 0, 22); fwrite(res, 1, 22, out); } /* reserved */

    /* Sample header (40 bytes) */
    writeU32(out, (unsigned int)smpLen);       /* sample length in bytes */
    writeU32(out, xmLoopStart);               /* loop start */
    writeU32(out, xmLoopLen);                 /* loop length */
    fputc(64, out);                           /* default volume (64 = max) */
    fputc(0, out);                            /* finetune (signed, 0 = none) */
    fputc(loopType, out);                     /* 0=none, 1=forward, 2=ping-pong */
    fputc(instSamplePan[inst], out);           /* panning from tracked channel usage */
    fputc((unsigned char)relNote, out);       /* relative note */
    fputc(0, out);                            /* reserved / encoding (0 = delta) */
    memset(instName, 0, 22);
    snprintf(instName, 22, "%s", labelBuf);
    fwrite(instName, 1, 22, out);

    /* Write delta-encoded sample data immediately after sample header */
    unsigned char* delta = (unsigned char*)malloc((size_t)smpLen);
    if (delta) {
      deltaEncode8(smpl + smpStart, smpLen, delta);
      fwrite(delta, 1, (size_t)smpLen, out);
      free(delta);
    }
  }

  fclose(out);
  free(mdatBuf);
  free(smpl);
  printf("Wrote %s\n", outPath);
  return true;
}
