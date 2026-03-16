/*
 * TFMX to XM converter.
 * Reads TFMX mdat+smpl and writes an XM (Extended Module) file.
 * Layout follows: The Unofficial XM File Format Specification (Kamenar, CelerSMS).
 * - Header size 276; [Header size]+60 = offset of 1st pattern header (4 dword length, then 9 bytes, then data).
 * - Instrument size 263 (example 07 01 00 00 in spec); sample header 40 bytes at end of block.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef _WIN32
#include <arpa/inet.h>
#endif

#include "tfmx.h"

#ifndef TFMXPLAY_VERSION
#define TFMXPLAY_VERSION "0.0.1"
#endif

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

struct MacroSampleInfo {
  int start;       // byte offset in smpl file
  int len;         // length in bytes (8-bit samples)
  int loopStart;   // loop start in bytes relative to sample start
  int loopLen;     // loop length in bytes (0 = no loop)
  bool oneShot;    // true if mOneShot found
  bool hasSetLoop; // true if mSetLoop found
  int addNote;     // cumulative AddNote transposition from macro
};

struct MacroEffectInfo {
  int addVol;         // from mAddVol data[2] (SIGNED), or -999 if absent
  int setVol;         // from mSetVol data[2], or -1
  int vibSpeed;       // XM vibrato speed x (1-15), 0 = no vibrato
  int vibDepth;       // XM vibrato depth y (1-15)
  int sustainSlide;   // sustain envelope: volume slide before key-up
  int sustainDelay;   // rows to wait before sustain envelope starts
  int releaseSlide;   // release envelope: volume slide after key-up (fade to 0)
};

static MacroSampleInfo getMacroSample(const TFMXMacroData macro[256]) {
  MacroSampleInfo info = { 0, 512, 0, 0, false, false, 0 };
  int curStart = 0;
  int curLenWords = 256;
  bool gotAny = false;
  bool gotAddNote = false;
  bool afterOn = false;

  /* Track two phases: at-mOn (attack) and post-mOn (sustain).
   * Multi-phase macros (e.g. drum attack → bass sustain) set new
   * mSetBegin/mSetLen AFTER mOn. We prefer the post-mOn (sustained)
   * sample because it represents the main body of the sound. */
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

static int calcEnvSpeed(int envAmt, int envTimeC, int xmSpeed) {
  int denom = (envTimeC + 1) * (xmSpeed > 1 ? (xmSpeed - 1) : 1);
  int speed = (envAmt * xmSpeed + denom - 1) / denom;
  if (speed < 1) speed = 1;
  if (speed > 15) speed = 15;
  return speed;
}

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
  if (fx.releaseSlide == 0 && foundSustainEnv && fx.sustainSlide < 0) {
    fx.releaseSlide = fx.sustainSlide;
  }
  return fx;
}

/* Delta-encode 8-bit sample for XM. */
static void deltaEncode8(const signed char* src, int len, unsigned char* dst) {
  int prev = 0;
  for (int i = 0; i < len; i++) {
    int s = (unsigned char)src[i];
    dst[i] = (unsigned char)((s - prev) & 0xff);
    prev = s;
  }
}

/* Read a TFMX pattern entry from raw mdat buffer at given absolute file offset. */
static TFMXPatData readPatEntry(const unsigned char* mdatBuf, size_t mdatSize, int fileOffset) {
  TFMXPatData item;
  if (fileOffset < 0 || fileOffset + 4 > (int)mdatSize) {
    item.note = 0xF0; /* pEnd safety */
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

  /* Load entire mdat file into raw buffer */
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

  /* Parse header from buffer */
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
  if (head.ordSeek == 0) head.ordSeek = 0x800;
  if (head.patSeek == 0) head.patSeek = 0x400;
  if (head.macroSeek == 0) head.macroSeek = 0x600;

  /* Parse pattern pointers from buffer */
  for (int i = 0; i < 128; i++) {
    int off = head.patSeek + i * 4;
    if (off + 4 <= (int)mdatSize)
      patPoint[i] = (int)readBE32(mdatBuf, off);
    else
      patPoint[i] = 0;
  }

  /* Parse macro pointers from buffer */
  for (int i = 0; i < 128; i++) {
    int off = head.macroSeek + i * 4;
    if (off + 4 <= (int)mdatSize)
      macroPoint[i] = (int)readBE32(mdatBuf, off);
    else
      macroPoint[i] = 0;
  }

  /* Parse order table from buffer */
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

  /* Load macros from buffer */
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
  for (int i = 0; i < 20 && head.desc[i] != '\0'; i++) {
    unsigned char c = (unsigned char)head.desc[i];
    xmName[i] = (c >= 32 && c < 127) ? c : ' ';
  }
  xmName[20] = '\0';
  fwrite(xmName, 1, 20, out);
  fputc(0x1A, out);
  memset(xmName, 0, 20);
  snprintf(xmName, 21, "tfmxplay %s", TFMXPLAY_VERSION);
  fwrite(xmName, 1, 20, out);
  writeU16(out, 0x0104);
  writeU32(out, 276);

  /* --- Simulate TFMX playback --- */
  const int ROWS_PER_PAT = 64;
  const int MAX_TOTAL_ROWS = ROWS_PER_PAT * 256;
  int numChans = 8;

  struct XMCell { unsigned char note, inst, vol, fx, param; };
  XMCell* bigGrid = (XMCell*)calloc((size_t)MAX_TOTAL_ROWS * numChans, sizeof(XMCell));
  if (!bigGrid) { fclose(out); free(mdatBuf); free(smpl); return false; }

  struct TrackState {
    int baseOff;       // absolute file offset from patPoint[patIdx], -1 if inactive
    int pos;           // step index within current pattern
    int tim;
    signed char trans;
    int loopCount;
    int savedBaseOff;  // saved by pGsPt
    int savedPos;      // saved by pGsPt
    bool inGosub;
  };
  TrackState ts[8];
  for (int t = 0; t < 8; t++) {
    ts[t].baseOff = -1; ts[t].pos = -1;
    ts[t].tim = 0x7fffffff; ts[t].inGosub = false;
  }

  /* Per-channel pattern-envelope state (from pEnve commands) */
  struct ChanEnv {
    int amt, timeReset, timeC, target;
    bool active;
    int vol; /* current volume 0-64 */
  };
  ChanEnv chanEnv[8];
  for (int c = 0; c < 8; c++) chanEnv[c].active = false;

  int totalXmRows = 0;

  /* Initialize Amiga stereo panning on row 0: L,R,R,L,L,R,R,L */
  for (int ch = 0; ch < numChans; ch++) {
    bool isRight = (ch & 1) ^ ((ch & 2) >> 1);
    bigGrid[ch].fx = 8;
    bigGrid[ch].param = isRight ? 0xD0 : 0x30;
  }

  int xmSpeed = head.songSpeed[subsong] + 1;
  if (xmSpeed < 1) xmSpeed = 6;
  if (xmSpeed > 31) xmSpeed = 31;

  MacroEffectInfo macroFX[128];
  for (int i = 0; i < 128; i++) macroFX[i] = getMacroEffects(macro[i], xmSpeed);

  int startRow = head.songStart[subsong];
  int endRow = head.songEnd[subsong];
  if (startRow < 0) startRow = 0;
  if (endRow < startRow) endRow = startRow;

  auto initOrderRow = [&](int ordRow) {
    for (int t = 0; t < 8; t++) {
      unsigned char p = track[ordRow][t].pat;
      if (p == 0x80) continue; /* continue current pattern */
      if (p == 0xFE) {
        /* Channel-off: silence the target channel */
        int ch = track[ordRow][t].trans;
        if (ch >= 0 && ch < numChans && totalXmRows < MAX_TOTAL_ROWS) {
          bigGrid[totalXmRows * numChans + ch].vol = 0x10; /* volume 0 */
        }
        ts[t].baseOff = -1;
        ts[t].tim = 0x7fffffff;
        continue;
      }
      if (p >= 0x80) { /* 0xFF = stop, other high values = inactive */
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

  for (int ordRow = startRow; ordRow <= endRow && totalXmRows < MAX_TOTAL_ROWS; ordRow++) {
    if (track[ordRow][0].pat == 0xEF && track[ordRow][0].trans == 0xFE) {
      int cmd = track[ordRow][1].trans;
      if (cmd == 0) break; /* stop */
      if (cmd == 1) {
        int jumpTarget = track[ordRow][2].trans;
        if (jumpTarget <= ordRow) break;
        ordRow = jumpTarget - 1;
      }
      /* cmd == 2: tempo change — apply speed from data */
      if (cmd == 2) {
        /* tempo is stored in track[ordRow][3] as (pat<<8)|trans */
      }
      continue;
    }

    initOrderRow(ordRow);

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
              if (ts[t].inGosub) {
                ts[t].baseOff = ts[t].savedBaseOff;
                ts[t].pos = ts[t].savedPos;
                ts[t].inGosub = false;
              } else {
                rowDone = true; getMeOut = true;
              }
              break;
            case pStop:
              ts[t].baseOff = -1;
              ts[t].tim = 0x7fffffff; getMeOut = true;
              break;
            case pWait:
              ts[t].tim = item.ins; getMeOut = true;
              break;
            case pLoop: {
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
              if (item.ins < 128)
                ts[t].baseOff = patPoint[item.ins];
              ts[t].pos = ((item.vol << 12) | (item.chan << 8) | item.detune) - 1;
              ts[t].loopCount = 0;
              break;
            case pGsPt: {
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
              if (ts[t].inGosub) {
                ts[t].baseOff = ts[t].savedBaseOff;
                ts[t].pos = ts[t].savedPos;
                ts[t].inGosub = false;
              }
              break;
            case pEnve: {
              int ch = item.chan;
              if (ch >= 0 && ch < numChans) {
                chanEnv[ch].amt = item.ins;
                chanEnv[ch].timeReset = item.vol + 1;
                chanEnv[ch].timeC = item.vol + 1;
                chanEnv[ch].target = item.detune;
                chanEnv[ch].active = true;
              }
              break;
            }
            case pKeyUp: {
              int ch = item.chan;
              if (ch >= 0 && ch < numChans && totalXmRows < MAX_TOTAL_ROWS)
                bigGrid[totalXmRows * numChans + ch].note = 97;
              break;
            }
            case pNOP: break;
            default:
              if (item.note >= 0xf0) break; /* other unhandled Fxx commands */
              if ((item.note & 0xc0) == 0xc0) break; /* portamento — skip */

              { int rawNote = (item.note & 0x3f) + ts[t].trans;
                int xmNote = rawNote + 25;
                if (xmNote < 1) xmNote = 1;
                if (xmNote > 96) xmNote = 96;
                int ch = item.chan;
                if (ch < 0 || ch >= numChans) ch = 0;
                if (totalXmRows < MAX_TOTAL_ROWS) {
                  XMCell* cell = &bigGrid[totalXmRows * numChans + ch];
                  cell->note = (unsigned char)xmNote;
                  cell->inst = (unsigned char)(item.ins + 1);
                  int v = item.vol;
                  cell->vol = (unsigned char)(0x10 + v * 4);
                  if (cell->vol > 0x50) cell->vol = 0x50;
                  /* Init envelope volume to effective TFMX volume */
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
                }
                /* Notes with top bits 10 (0x80-0xBF) OR 0x7F set wait */
                if ((item.note & 0xc0) == 0x80 || item.note == 0x7f) {
                  ts[t].tim = item.detune;
                  getMeOut = true;
                }
              }
              break;
          }
        }
      }
      /* Apply per-channel pattern envelopes (pEnve) each tick */
      if (totalXmRows < MAX_TOTAL_ROWS) {
        for (int c = 0; c < numChans; c++) {
          if (!chanEnv[c].active) continue;
          if (--chanEnv[c].timeC <= 0) {
            chanEnv[c].timeC = chanEnv[c].timeReset;
            if (chanEnv[c].vol < chanEnv[c].target)
              chanEnv[c].vol = (chanEnv[c].vol + chanEnv[c].amt < chanEnv[c].target)
                                ? chanEnv[c].vol + chanEnv[c].amt : chanEnv[c].target;
            else if (chanEnv[c].vol > chanEnv[c].target)
              chanEnv[c].vol = (chanEnv[c].vol - chanEnv[c].amt > chanEnv[c].target)
                                ? chanEnv[c].vol - chanEnv[c].amt : chanEnv[c].target;
            if (chanEnv[c].vol == chanEnv[c].target)
              chanEnv[c].active = false;
          }
          XMCell* cell = &bigGrid[totalXmRows * numChans + c];
          if (cell->note == 0 && cell->inst == 0 && cell->vol == 0) {
            int v = chanEnv[c].vol;
            if (v < 0) v = 0;
            if (v > 64) v = 64;
            cell->vol = (unsigned char)(0x10 + v);
          }
        }
      }
      if (!anyActive) rowDone = true;
      if (!rowDone && totalXmRows < MAX_TOTAL_ROWS) totalXmRows++;
    }
  }
  if (totalXmRows < 1) totalXmRows = 1;

  /* --- Post-process: vibrato, envelope fades, volume correction --- */

  for (int ch = 0; ch < numChans; ch++) {
    bool vibActive = false;
    bool vibPending = false;
    int envSlide = 0;
    int envDelayLeft = 0;
    int releaseSlide = 0;
    bool inRelease = false;
    int curVibS = 0, curVibD = 0;
    bool isRight = (ch & 1) ^ ((ch & 2) >> 1);
    unsigned char panParam = isRight ? 0xD0 : 0x30;

    for (int row = 0; row < totalXmRows; row++) {
      XMCell* cell = &bigGrid[row * numChans + ch];

      if (cell->note > 0 && cell->note < 97 && cell->inst > 0) {
        int idx = cell->inst - 1;
        if (idx >= 0 && idx < 128) {
          MacroEffectInfo& ef = macroFX[idx];

          int rawPV = ((int)cell->vol - 0x10);
          if (rawPV < 0) rawPV = 0;
          int patVol = rawPV / 4;
          int effVol;
          if (ef.setVol >= 0)        effVol = ef.setVol;
          else if (ef.addVol != -999) effVol = ef.addVol + patVol * 3;
          else                        effVol = patVol * 3;
          if (effVol > 64) effVol = 64;
          if (effVol < 0) effVol = 0;
          cell->vol = (unsigned char)(0x10 + effVol);

          cell->fx = 8;
          cell->param = panParam;

          if (ef.vibSpeed > 0 && ef.vibDepth > 0) {
            vibPending = true;
            vibActive = true;
            curVibS = ef.vibSpeed; curVibD = ef.vibDepth;
          } else {
            vibActive = false;
            vibPending = false;
          }

          envSlide = ef.sustainSlide;
          envDelayLeft = ef.sustainDelay;
          releaseSlide = ef.releaseSlide;
          inRelease = false;
        }
      } else if (cell->note == 97) {
        cell->note = 0;
        if (releaseSlide != 0) {
          envSlide = releaseSlide;
          envDelayLeft = 0;
          inRelease = true;
        } else if (envSlide < 0) {
          inRelease = true;
        } else {
          envSlide = -2;
          inRelease = true;
        }
        vibActive = false;
        vibPending = false;
      } else if (cell->note == 0 && cell->inst == 0) {
        if (vibPending && cell->fx == 0) {
          cell->fx = 4;
          cell->param = (unsigned char)((curVibS << 4) | curVibD);
          vibPending = false;
        } else if (vibActive && cell->fx == 0) {
          cell->fx = 4;
          cell->param = 0;
        }
        if (envSlide != 0 && cell->vol == 0) {
          if (envDelayLeft > 0) {
            envDelayLeft--;
          } else {
            if (envSlide < 0)
              cell->vol = (unsigned char)(0x60 + (-envSlide));
            else if (envSlide > 0)
              cell->vol = (unsigned char)(0x70 + envSlide);
          }
        }
      }
    }
  }

  int numPatterns = (totalXmRows + ROWS_PER_PAT - 1) / ROWS_PER_PAT;
  if (numPatterns < 1) numPatterns = 1;
  if (numPatterns > 256) numPatterns = 256;
  int songLen = numPatterns;

  printf("XM-Export: %d total rows -> %d patterns (%d rows/pat), speed=%d BPM=125\n",
         totalXmRows, numPatterns, ROWS_PER_PAT, xmSpeed);

  writeU16(out, (unsigned short)songLen);
  writeU16(out, 0);
  writeU16(out, 8);
  writeU16(out, (unsigned short)numPatterns);
  writeU16(out, 128);
  writeU16(out, 1);
  writeU16(out, (unsigned short)xmSpeed);
  writeU16(out, 125);
  unsigned char order[256];
  memset(order, 0, 256);
  for (int i = 0; i < numPatterns && i < 256; i++) order[i] = (unsigned char)i;
  fwrite(order, 1, 256, out);

  /* Write patterns */
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

  /* Instruments: 128 instruments, each 1 sample */
  for (int inst = 0; inst < 128; inst++) {
    MacroSampleInfo si = getMacroSample(macro[inst]);
    if (si.start + si.len > (int)smplLen) {
      si.start = 0;
      si.len = 512;
      si.oneShot = true;
    }
    if (si.len < 4) si.len = 4;

    unsigned char loopType = 0;
    unsigned int xmLoopStart = 0;
    unsigned int xmLoopLen = 0;
    if (si.oneShot) {
      loopType = 0;
    } else if (si.hasSetLoop) {
      loopType = 1;
      xmLoopStart = (unsigned int)si.loopStart;
      xmLoopLen = (unsigned int)si.loopLen;
      if (xmLoopStart + xmLoopLen > (unsigned int)si.len) {
        xmLoopLen = (unsigned int)si.len - xmLoopStart;
      }
      if (xmLoopLen < 2) { xmLoopLen = 2; }
    } else {
      loopType = 1;
      xmLoopStart = 0;
      xmLoopLen = (unsigned int)si.len;
    }

    writeU32(out, 263);

    char instName[22];
    memset(instName, 0, 22);
    snprintf(instName, 22, "Instrument %03d", inst + 1);
    fwrite(instName, 1, 22, out);
    fputc(0, out);
    writeU16(out, 1);

    writeU32(out, 40);
    for (int i = 0; i < 96; i++) fputc(0, out);
    for (int i = 0; i < 12; i++) { writeU16(out, 0); writeU16(out, 0); }
    for (int i = 0; i < 12; i++) { writeU16(out, 0); writeU16(out, 32); }
    fputc(0, out); fputc(0, out);
    fputc(0, out); fputc(0, out); fputc(0, out);
    fputc(0, out); fputc(0, out); fputc(0, out);
    fputc(0, out); fputc(0, out);
    fputc(0, out); fputc(0, out); fputc(0, out); fputc(0, out);
    writeU16(out, 256);
    { unsigned char res[22]; memset(res, 0, 22); fwrite(res, 1, 22, out); }

    writeU32(out, (unsigned int)si.len);
    writeU32(out, xmLoopStart);
    writeU32(out, xmLoopLen);
    fputc(64, out);
    fputc(0, out);
    fputc(loopType, out);
    fputc(128, out);
    fputc((unsigned char)(signed char)si.addNote, out);
    fputc(0, out);
    memset(instName, 0, 22);
    snprintf(instName, 22, "Sample %03d", inst + 1);
    fwrite(instName, 1, 22, out);

    unsigned char* delta = (unsigned char*)malloc((size_t)si.len);
    if (delta) {
      deltaEncode8(smpl + si.start, si.len, delta);
      fwrite(delta, 1, (size_t)si.len, out);
      free(delta);
    }
  }

  fclose(out);
  free(mdatBuf);
  free(smpl);
  printf("Wrote %s\n", outPath);
  return true;
}
