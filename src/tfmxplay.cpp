#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <signal.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#endif
#include <string>
#include <vector>
#ifdef _WIN32
#include <SDL.h>
#else
#include <SDL2/SDL.h>
#endif
#ifdef _SYNC_VBLANK
#ifdef _WIN32
#else
#include <fcntl.h>
#include <libdrm/drm.h>
#endif
#endif

#include "ta-time.h"

#ifndef TFMXPLAY_VERSION
#define TFMXPLAY_VERSION "0.0.5"
#endif

#include "blip_buf.h"
#include "tfmx.h"
#include "convert2xm.h"

typedef std::string string;

struct Param {
  string shortName;
  string name;
  string valName;
  string desc;
  bool value;
  bool (*func)(string);
  Param(string sn, string n, bool v, bool (*f)(string), string vn, string d): shortName(sn), name(n), valName(vn), desc(d), value(v), func(f) {}
};

std::vector<Param> params;

bool needsValue(string param) {
  for (size_t i=0; i<params.size(); i++) {
    if (params[i].name==param || params[i].shortName==param) {
      return params[i].value;
    }
  }
  return false;
}

blip_buffer_t* bb[2];
int prevSample[2]={0,0};
short bbOut[2][32768];

SDL_AudioDeviceID ai;
SDL_AudioSpec ac;
SDL_AudioSpec ar;

bool quit, ntsc, hle, dumpFile, showVersionOnly, showInfo, doConvert2XM;
std::string convert2xmPath;
PanPreset panPreset=PAN_SOFT;
std::string muteChanStr;
FILE* dump;
long dumpDataSize;

static void wavWriteLE32(FILE* f, unsigned int v) {
  unsigned char b[4]={(unsigned char)(v&0xff),(unsigned char)((v>>8)&0xff),(unsigned char)((v>>16)&0xff),(unsigned char)((v>>24)&0xff)};
  fwrite(b,1,4,f);
}

static void wavWriteLE16(FILE* f, unsigned short v) {
  unsigned char b[2]={(unsigned char)(v&0xff),(unsigned char)((v>>8)&0xff)};
  fwrite(b,1,2,f);
}

static void wavWriteHeader(FILE* f, int sampleRate, int channels, int bitsPerSample) {
  int byteRate=sampleRate*channels*(bitsPerSample/8);
  int blockAlign=channels*(bitsPerSample/8);
  fwrite("RIFF",1,4,f);
  wavWriteLE32(f,0); // placeholder for file size - 8
  fwrite("WAVE",1,4,f);
  fwrite("fmt ",1,4,f);
  wavWriteLE32(f,16); // PCM fmt chunk size
  wavWriteLE16(f,1);  // PCM format
  wavWriteLE16(f,(unsigned short)channels);
  wavWriteLE32(f,(unsigned int)sampleRate);
  wavWriteLE32(f,(unsigned int)byteRate);
  wavWriteLE16(f,(unsigned short)blockAlign);
  wavWriteLE16(f,(unsigned short)bitsPerSample);
  fwrite("data",1,4,f);
  wavWriteLE32(f,0); // placeholder for data size
}

static void wavFinalizeHeader(FILE* f, long dataSize) {
  fseek(f,4,SEEK_SET);
  wavWriteLE32(f,(unsigned int)(dataSize+36));
  fseek(f,40,SEEK_SET);
  wavWriteLE32(f,(unsigned int)dataSize);
  fseek(f,0,SEEK_END);
}

int sr, speed;
double targetSR;
int songid;

TFMXPlayer p;

int defCIAVal;

#ifdef _WIN32
#else
struct sigaction intsa;
struct termios termprop;
struct termios termpropold;
#endif

#ifdef _SYNC_VBLANK
int syncfd;
bool syncVBlank;
#endif

const char* truth[]={
  "false", "true"
};

void finish() {
#ifndef _WIN32
  if (tcsetattr(0,TCSAFLUSH,&termpropold)!=0) {
    printf("WARNING: FAILURE TO SET FLAGS TO QUIT!\n");
    return;
  }
#endif
}

static void handleTerm(int) {
  quit=true;
  printf("quit!\n");
  SDL_CloseAudioDevice(ai);
  finish();
  if (dumpFile) {
    printf("closing dump\n");
    fflush(dump);
    wavFinalizeHeader(dump,dumpDataSize);
    fclose(dump);
    dumpFile=false;
  }
  exit(0);
}

#ifdef _WIN32
static BOOL WINAPI handleConsoleCtrl(DWORD ctrlType) {
  if (ctrlType==CTRL_C_EVENT || ctrlType==CTRL_BREAK_EVENT) {
    quit=true;
    printf("quit!\n");
    SDL_CloseAudioDevice(ai);
    finish();
    if (dumpFile) {
      printf("closing dump\n");
      fflush(dump);
      wavFinalizeHeader(dump,dumpDataSize);
      fclose(dump);
      dumpFile=false;
    }
    exit(0);
    return TRUE;
  }
  return FALSE;
}
#endif

static void processHLE(void*, Uint8* stream, int len) {
  short* buf[2];
  short temp[2];
  unsigned int nframes=len/(2*ar.channels);
  buf[0]=(short*)stream;
  buf[1]=&buf[0][1];

  size_t runtotal=nframes;

  for (size_t i=0; i<runtotal; i++) {
    p.nextSampleHLE(&temp[0],&temp[1]);

    buf[0][i*ar.channels]=(temp[0]+(temp[1]>>2))<<1;
    buf[1][i*ar.channels]=(temp[1]+(temp[0]>>2))<<1;
    //buf[0][i*ar.channels]=temp[0];
    //buf[1][i*ar.channels]=temp[1];
  }
  if (dumpFile) {
    size_t written=fwrite(stream,1,len,dump);
    if (written>0) dumpDataSize+=written;
    if ((int)written<len) {
      perror("cannot write");
      printf("stopping dump!\n");
      wavFinalizeHeader(dump,dumpDataSize);
      fclose(dump);
      dumpFile=false;
    }
  }
}

static void process(void*, Uint8* stream, int len) {
  short* buf[2];
  short temp[2];
  unsigned int nframes=len/(2*ar.channels);
  buf[0]=(short*)stream;
  buf[1]=&buf[0][1];
  
  blip_set_rates(bb[0],targetSR,sr);
  blip_set_rates(bb[1],targetSR,sr);
  p.hleRate=float((double)targetSR/(double)sr);
  
  size_t runtotal=blip_clocks_needed(bb[0],nframes);

  for (size_t i=0; i<runtotal; i++) {
    p.nextSample(&temp[0],&temp[1]);

    blip_add_delta(bb[0],i,(temp[0]+(temp[1]>>2)-prevSample[0])<<1);
    blip_add_delta(bb[1],i,(temp[1]+(temp[0]>>2)-prevSample[1])<<1);
    prevSample[0]=temp[0]+(temp[1]>>2);
    prevSample[1]=temp[1]+(temp[0]>>2);
  }

  blip_end_frame(bb[0],runtotal);
  blip_end_frame(bb[1],runtotal);

  blip_read_samples(bb[0],bbOut[0],nframes,0);
  blip_read_samples(bb[1],bbOut[1],nframes,0);

  for (size_t i=0; i<nframes; i++) {
    buf[0][i*ar.channels]=bbOut[0][i];
    buf[1][i*ar.channels]=bbOut[1][i];
  }

  if (dumpFile) {
    size_t written=fwrite(stream,1,len,dump);
    if (written>0) dumpDataSize+=written;
    if ((int)written<len) {
      perror("cannot write");
      printf("stopping dump!\n");
      wavFinalizeHeader(dump,dumpDataSize);
      fclose(dump);
      dumpFile=false;
    }
  }
}

void initConsole() {
#ifdef _WIN32
  SetConsoleOutputCP(65001);
  HANDLE hOut=GetStdHandle(STD_OUTPUT_HANDLE);
  if (hOut!=INVALID_HANDLE_VALUE) {
    DWORD mode=0;
    if (GetConsoleMode(hOut,&mode)) {
      SetConsoleMode(hOut,mode|ENABLE_VIRTUAL_TERMINAL_PROCESSING);
    }
  }
#endif
}

#define B "\x1b[34m"
#define R "\x1b[31m"
#define N "\x1b[m"
#define G "  "

void printBanner() {
  static const char* banner[] = {
    B "\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x95\x97\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x95\x97\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x95\x97   \xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x95\x97\xe2\x96\x88\xe2\x96\x88\xe2\x95\x97  \xe2\x96\x88\xe2\x96\x88\xe2\x95\x97\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x95\x97 \xe2\x96\x88\xe2\x96\x88\xe2\x95\x97      \xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x95\x97 \xe2\x96\x88\xe2\x96\x88\xe2\x95\x97   \xe2\x96\x88\xe2\x96\x88\xe2\x95\x97" N G R "\xe2\x96\x88\xe2\x96\x88\xe2\x95\x97  \xe2\x96\x88\xe2\x96\x88\xe2\x95\x97\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x95\x97   \xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x95\x97" N,
    B "\xe2\x95\x9a\xe2\x95\x90\xe2\x95\x90\xe2\x96\x88\xe2\x96\x88\xe2\x95\x94\xe2\x95\x90\xe2\x95\x90\xe2\x95\x9d\xe2\x96\x88\xe2\x96\x88\xe2\x95\x94\xe2\x95\x90\xe2\x95\x90\xe2\x95\x90\xe2\x95\x90\xe2\x95\x9d\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x95\x97 \xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x95\x91\xe2\x95\x9a\xe2\x96\x88\xe2\x96\x88\xe2\x95\x97\xe2\x96\x88\xe2\x96\x88\xe2\x95\x94\xe2\x95\x9d\xe2\x96\x88\xe2\x96\x88\xe2\x95\x94\xe2\x95\x90\xe2\x95\x90\xe2\x96\x88\xe2\x96\x88\xe2\x95\x97\xe2\x96\x88\xe2\x96\x88\xe2\x95\x91     \xe2\x96\x88\xe2\x96\x88\xe2\x95\x94\xe2\x95\x90\xe2\x95\x90\xe2\x96\x88\xe2\x96\x88\xe2\x95\x97\xe2\x95\x9a\xe2\x96\x88\xe2\x96\x88\xe2\x95\x97 \xe2\x96\x88\xe2\x96\x88\xe2\x95\x94\xe2\x95\x9d" N G R "\xe2\x95\x9a\xe2\x96\x88\xe2\x96\x88\xe2\x95\x97\xe2\x96\x88\xe2\x96\x88\xe2\x95\x94\xe2\x95\x9d\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x95\x97 \xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x95\x91" N,
    B "   \xe2\x96\x88\xe2\x96\x88\xe2\x95\x91   \xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x95\x97  \xe2\x96\x88\xe2\x96\x88\xe2\x95\x94\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x95\x94\xe2\x96\x88\xe2\x96\x88\xe2\x95\x91 \xe2\x95\x9a\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x95\x94\xe2\x95\x9d \xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x95\x94\xe2\x95\x9d\xe2\x96\x88\xe2\x96\x88\xe2\x95\x91     \xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x95\x91 \xe2\x95\x9a\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x95\x94\xe2\x95\x9d " N G R " \xe2\x95\x9a\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x95\x94\xe2\x95\x9d \xe2\x96\x88\xe2\x96\x88\xe2\x95\x94\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x95\x94\xe2\x96\x88\xe2\x96\x88\xe2\x95\x91" N,
    B "   \xe2\x96\x88\xe2\x96\x88\xe2\x95\x91   \xe2\x96\x88\xe2\x96\x88\xe2\x95\x94\xe2\x95\x90\xe2\x95\x90\xe2\x95\x9d  \xe2\x96\x88\xe2\x96\x88\xe2\x95\x91\xe2\x95\x9a\xe2\x96\x88\xe2\x96\x88\xe2\x95\x94\xe2\x95\x9d\xe2\x96\x88\xe2\x96\x88\xe2\x95\x91 \xe2\x96\x88\xe2\x96\x88\xe2\x95\x94\xe2\x96\x88\xe2\x96\x88\xe2\x95\x97 \xe2\x96\x88\xe2\x96\x88\xe2\x95\x94\xe2\x95\x90\xe2\x95\x90\xe2\x95\x90\xe2\x95\x9d \xe2\x96\x88\xe2\x96\x88\xe2\x95\x91     \xe2\x96\x88\xe2\x96\x88\xe2\x95\x94\xe2\x95\x90\xe2\x95\x90\xe2\x96\x88\xe2\x96\x88\xe2\x95\x91  \xe2\x95\x9a\xe2\x96\x88\xe2\x96\x88\xe2\x95\x94\xe2\x95\x9d  " N G R " \xe2\x96\x88\xe2\x96\x88\xe2\x95\x94\xe2\x96\x88\xe2\x96\x88\xe2\x95\x97 \xe2\x96\x88\xe2\x96\x88\xe2\x95\x91\xe2\x95\x9a\xe2\x96\x88\xe2\x96\x88\xe2\x95\x94\xe2\x95\x9d\xe2\x96\x88\xe2\x96\x88\xe2\x95\x91" N,
    B "   \xe2\x96\x88\xe2\x96\x88\xe2\x95\x91   \xe2\x96\x88\xe2\x96\x88\xe2\x95\x91     \xe2\x96\x88\xe2\x96\x88\xe2\x95\x91 \xe2\x95\x9a\xe2\x95\x90\xe2\x95\x9d \xe2\x96\x88\xe2\x96\x88\xe2\x95\x91\xe2\x96\x88\xe2\x96\x88\xe2\x95\x94\xe2\x95\x9d \xe2\x96\x88\xe2\x96\x88\xe2\x95\x97\xe2\x96\x88\xe2\x96\x88\xe2\x95\x91     \xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x95\x97\xe2\x96\x88\xe2\x96\x88\xe2\x95\x91  \xe2\x96\x88\xe2\x96\x88\xe2\x95\x91   \xe2\x96\x88\xe2\x96\x88\xe2\x95\x91   " N G R "\xe2\x96\x88\xe2\x96\x88\xe2\x95\x94\xe2\x95\x9d \xe2\x96\x88\xe2\x96\x88\xe2\x95\x97\xe2\x96\x88\xe2\x96\x88\xe2\x95\x91 \xe2\x95\x9a\xe2\x95\x90\xe2\x95\x9d \xe2\x96\x88\xe2\x96\x88\xe2\x95\x91" N,
    B "   \xe2\x95\x9a\xe2\x95\x90\xe2\x95\x9d   \xe2\x95\x9a\xe2\x95\x90\xe2\x95\x9d     \xe2\x95\x9a\xe2\x95\x90\xe2\x95\x9d     \xe2\x95\x9a\xe2\x95\x90\xe2\x95\x9d\xe2\x95\x9a\xe2\x95\x90\xe2\x95\x9d  \xe2\x95\x9a\xe2\x95\x90\xe2\x95\x9d\xe2\x95\x9a\xe2\x95\x90\xe2\x95\x9d     \xe2\x95\x9a\xe2\x95\x90\xe2\x95\x90\xe2\x95\x90\xe2\x95\x90\xe2\x95\x90\xe2\x95\x90\xe2\x95\x9d\xe2\x95\x9a\xe2\x95\x90\xe2\x95\x9d  \xe2\x95\x9a\xe2\x95\x90\xe2\x95\x9d   \xe2\x95\x9a\xe2\x95\x90\xe2\x95\x9d   " N G R "\xe2\x95\x9a\xe2\x95\x90\xe2\x95\x9d  \xe2\x95\x9a\xe2\x95\x90\xe2\x95\x9d\xe2\x95\x9a\xe2\x95\x90\xe2\x95\x9d     \xe2\x95\x9a\xe2\x95\x90\xe2\x95\x9d" N,
    NULL
  };
  printf("\n");
  for (int i=0; banner[i]; i++) printf("%s\n", banner[i]);
}

void printVersion() {
  printBanner();
  printf("\x1b[33mtfmxplay XM %s (c) 2026 by ray77@noisebay.org\x1b[m\n\n", TFMXPLAY_VERSION);
}

bool parHelp(string) {
  printVersion();
  printf("usage: tfmxplay [-params] mdat.file [smpl.file]\n");
  for (auto& i: params) {
    if (i.value) {
      printf("  -%s %s: %s\n",i.name.c_str(),i.valName.c_str(),i.desc.c_str());
    } else {
      printf("  -%s: %s\n",i.name.c_str(),i.desc.c_str());
    }
  }
  printf("Runtime keys:\n");
  printf("  Tab: next subsong\n");
  printf("  Shift+Tab: previous subsong\n");
  return false;
}

bool parNTSC(string) {
  ntsc=true;
  return true;
}

bool parSong(string v) {
  try {
    songid=std::stoi(v);
    if (songid<0 || songid>127) {
      printf("song number must be between 0 and 127.\n");
      return false;
    }
  } catch (std::exception& e) {
    printf("type a number, silly.\n");
    return false;
  }
  return true;
}

bool parHLE(string) {
  hle=true;
  return true;
}

#ifdef _SYNC_VBLANK
bool parVBlank(string) {
  syncVBlank=true;
  return true;
}
#endif

bool parDump(string) {
  dumpFile=true;
  return true;
}

bool parSpeed(string v) {
  try {
    defCIAVal=std::stoi(v);
  } catch (std::exception& e) {
    printf("type a number, silly.\n");
    return false;
  }
  return true;
}

bool parVersion(string) {
  showVersionOnly=true;
  return true;
}

bool parInfo(string) {
  showInfo=true;
  return true;
}

static unsigned short be16(const void* p) {
  const unsigned char* b=(const unsigned char*)p;
  return (unsigned short)((b[0]<<8)|b[1]);
}
static unsigned int be32(const void* p) {
  const unsigned char* b=(const unsigned char*)p;
  return ((unsigned int)b[0]<<24)|((unsigned int)b[1]<<16)|((unsigned int)b[2]<<8)|b[3];
}

void printInfo(const char* mdatPath) {
  FILE* f=fopen(mdatPath,"rb");
  if (!f) {
    printf("cannot open %s\n",mdatPath);
    return;
  }
  fseek(f,0,SEEK_END);
  long fileSize=ftell(f);
  fseek(f,0,SEEK_SET);

  unsigned char raw[512];
  if (fread(raw,1,512,f)!=512) {
    printf("file too small for TFMX header\n");
    fclose(f);
    return;
  }
  fclose(f);

  char ident[11];
  memcpy(ident,raw,10);
  ident[10]=0;

  const char* formatName="unknown";
  if (memcmp(raw,"TFMX-SONG",9)==0) {
    formatName="TFMX (standard)";
  } else if (memcmp(raw,"TFMX_SONG",9)==0) {
    formatName="TFMX 7.0";
  }

  printf("\x1b[1;36mFile:\x1b[0m    %s\n",mdatPath);
  printf("\x1b[1;36mSize:\x1b[0m    %ld bytes\n",fileSize);
  printf("\x1b[1;36mFormat:\x1b[0m  %s (ident: \"%s\")\n",formatName,ident);

  unsigned int ordSeek=be32(&raw[0x1D0]);
  unsigned int patSeek=be32(&raw[0x1D4]);
  unsigned int macroSeek=be32(&raw[0x1D8]);
  bool packed=(ordSeek!=0);
  if (ordSeek==0) ordSeek=0x800;
  if (patSeek==0) patSeek=0x400;
  if (macroSeek==0) macroSeek=0x600;
  printf("\x1b[1;36mLayout:\x1b[0m  %s (ords=0x%X pats=0x%X macros=0x%X)\n",
    packed?"packed":"unpacked",ordSeek,patSeek,macroSeek);

  char desc[241];
  memcpy(desc,&raw[16],240);
  desc[240]=0;
  for (int i=239; i>=0 && (desc[i]==' '||desc[i]==0); i--) desc[i]=0;

  printf("\n\x1b[1;33mText:\x1b[0m\n");
  if (desc[0]) {
    int pos=0;
    while (pos<240 && desc[pos]) {
      int lineEnd=pos;
      while (lineEnd<pos+40 && lineEnd<240 && desc[lineEnd]) lineEnd++;
      char line[41];
      int len=lineEnd-pos;
      memcpy(line,&desc[pos],len);
      line[len]=0;
      for (int i=len-1; i>=0 && line[i]==' '; i--) line[i]=0;
      if (line[0]) printf("  %s\n",line);
      pos+=40;
    }
  } else {
    printf("  (empty)\n");
  }

  printf("\n\x1b[1;33mSubsongs:\x1b[0m\n");
  int count=0;
  for (int i=0; i<32; i++) {
    unsigned short start=be16(&raw[0x100+i*2]);
    unsigned short end=be16(&raw[0x140+i*2]);
    unsigned short spd=be16(&raw[0x180+i*2]);
    if (end<start) continue;
    if (start==0 && end==0 && i>0) continue;
    if (start>=128 || end>=128) continue;
    int rows=end-start+1;
    int speed=spd+1;
    const char* tempoType="ticks/row";
    if (spd>15) tempoType="BPM";
    printf("  \x1b[1m%2d:\x1b[0m rows %3d-%3d (%3d rows), speed %d %s\n",
      i,(int)start,(int)end,rows,speed,tempoType);
    count++;
  }
  if (count==0) printf("  (none found)\n");
  else printf("\n  %d subsong(s) found.\n",count);

  printf("\n");
}

bool parConvert2XM(string v) {
  doConvert2XM=true;
  convert2xmPath=v;
  return true;
}

bool parPan(string v) {
  if (v=="Amiga" || v=="amiga") panPreset=PAN_AMIGA;
  else if (v=="Soft" || v=="soft") panPreset=PAN_SOFT;
  else if (v=="Headphone" || v=="headphone") panPreset=PAN_HEADPHONE;
  else {
    printf("unknown pan preset: %s (use Amiga, Soft, or Headphone)\n",v.c_str());
    return false;
  }
  return true;
}

bool parMute(string v) {
  muteChanStr=v;
  return true;
}

void initParams() {
  params.push_back(Param("h","help",false,parHelp,"","display this help"));
  params.push_back(Param("v","version",false,parVersion,"","show version"));
  params.push_back(Param("i","info",false,parInfo,"","show TFMX module info (header, subsongs, format)"));

  params.push_back(Param("s","subsong",true,parSong,"(num)","select song"));
  params.push_back(Param("n","ntsc",false,parNTSC,"","use NTSC rate"));
  params.push_back(Param("l","hle",false,parHLE,"","use high-level emulation (lower quality but much faster)"));
  params.push_back(Param("d","dump",false,parDump,"","dump 16-bit stereo output to tfmx.wav"));
  params.push_back(Param("S","speed",true,parSpeed,"","set speed in clock/2 cycles"));
  params.push_back(Param("c","convert2xm",false,parConvert2XM,"[=file.xm]","convert TFMX to XM file (default: tfmx_<name>.xm)"));
  params.push_back(Param("p","pan",true,parPan,"(preset)","set XM panning (only with -convert2xm): Amiga (hard L/R), Soft (default, near-original), Headphone (reduced stereo)"));
  params.push_back(Param("M","mute",true,parMute,"(channels)","mute channels (e.g. -M 123 mutes ch 1,2,3)"));

#ifdef _SYNC_VBLANK
  params.push_back(Param("V","vblank",false,parVBlank,"","sync to VBlank"));
#endif
}

int main(int argc, char** argv) {
  initConsole();
  string mdat, smpl;
  defCIAVal=0;
  ntsc=false;
  dumpFile=false;
  dumpDataSize=0;
  showVersionOnly=false;
  showInfo=false;
  doConvert2XM=false;
  convert2xmPath="";
  songid=0;
#ifdef _SYNC_VBLANK
  syncVBlank=false;
#endif

  initParams();

  // parse arguments
  string arg, val;
  size_t eqSplit, argStart;
  for (int i=1; i<argc; i++) {
    arg=""; val="";
    if (argv[i][0]=='-') {
      if (argv[i][1]=='-') {
        argStart=2;
      } else {
        argStart=1;
      }
      arg=&argv[i][argStart];
      eqSplit=arg.find_first_of('=');
      if (eqSplit==string::npos) {
        if (needsValue(arg)) {
          if ((i+1)<argc) {
            val=argv[i+1];
            i++;
          } else {
            printf("incomplete param %s.\n",arg.c_str());
            return 1;
          }
        }
      } else {
        val=arg.substr(eqSplit+1);
        arg=arg.substr(0,eqSplit);
      }
      //printf("arg %s. val %s\n",arg.c_str(),val.c_str());
      for (size_t j=0; j<params.size(); j++) {
        if (params[j].name==arg || params[j].shortName==arg) {
          if (!params[j].func(val)) return 1;
          break;
        }
      }
    } else {
      if (mdat=="") {
        mdat=argv[i];
      } else {
        if (smpl=="") {
          smpl=argv[i];
        }
      }
    }
  }

  if (showVersionOnly) {
    printVersion();
    return 0;
  }

  if (mdat=="" && !showInfo) {
    printVersion();
    printf("usage: %s [-params] mdat.file [smpl.file]\n",argv[0]);
    return 1;
  }

  if (showInfo) {
    if (mdat=="") {
      printf("please provide a mdat file to inspect.\n");
      return 1;
    }
    printVersion();
    printInfo(mdat.c_str());
    return 0;
  }

  if (smpl=="") {
    size_t repPos=mdat.rfind("mdat");
    if (repPos==string::npos) {
      printf("cannot auto-locate smpl file. please provide it manually.\n");
      return 1;
    }
    smpl=mdat;
    smpl.replace(repPos,4,"smpl");
  }

  if (doConvert2XM) {
    printVersion();
    string outPath=convert2xmPath;
    if (outPath.empty()) {
      string base=mdat;
      size_t slash=base.find_last_of("/\\");
      if (slash!=string::npos) base=base.substr(slash+1);
      if (base.substr(0,5)=="mdat.") base=base.substr(5);
      else if (base.substr(0,4)=="mdat") base=base.substr(4);
      if (!base.empty() && base[0]=='_') base=base.substr(1);
      if (base.empty()) base="output";
      outPath="tfmx_"+base+".xm";
    }
    if (!convertToXM(mdat.c_str(),smpl.c_str(),outPath.c_str(),songid,panPreset)) {
      return 1;
    }
    return 0;
  }

  if (!p.load(mdat.c_str(),smpl.c_str())) {
    printf("could not open song...\n");
    return 1;
  }

  for (size_t mi=0; mi<muteChanStr.size(); mi++) {
    int mc=muteChanStr[mi]-'0';
    if (mc>=0 && mc<8) { p.mute(mc); printf("muting channel %d\n",mc); }
  }

  if (dumpFile && !muteChanStr.empty()) {
    sr=44100;
    if (ntsc) { targetSR=3579545; speed=59659; }
    else      { targetSR=3546895; speed=70937; }
    if (defCIAVal) p.setCIAVal(defCIAVal); else p.setCIAVal(speed);

    bb[0]=blip_new(32768);
    bb[1]=blip_new(32768);
    blip_set_rates(bb[0],targetSR,sr);
    blip_set_rates(bb[1],targetSR,sr);
    p.hleRate=float((double)targetSR/(double)sr);

    dump=fopen("tfmx.wav","wb");
    if (!dump) { perror("cannot dump"); return 1; }
    dumpDataSize=0;
    wavWriteHeader(dump,sr,2,16);

    p.play(songid);
    int renderFrames=sr*10;
    short outBuf[2048];
    int pos=0;
    printf("headless render: %d samples...\n",renderFrames);
    while (pos<renderFrames) {
      int chunk=1024; if (pos+chunk>renderFrames) chunk=renderFrames-pos;
      blip_set_rates(bb[0],targetSR,sr);
      blip_set_rates(bb[1],targetSR,sr);
      size_t runtotal=blip_clocks_needed(bb[0],chunk);
      short temp[2];
      for (size_t i=0; i<runtotal; i++) {
        p.nextSample(&temp[0],&temp[1]);
        blip_add_delta(bb[0],i,(temp[0]+(temp[1]>>2)-prevSample[0])<<1);
        blip_add_delta(bb[1],i,(temp[1]+(temp[0]>>2)-prevSample[1])<<1);
        prevSample[0]=temp[0]+(temp[1]>>2);
        prevSample[1]=temp[1]+(temp[0]>>2);
      }
      blip_end_frame(bb[0],runtotal);
      blip_end_frame(bb[1],runtotal);
      short bbL[1024],bbR[1024];
      blip_read_samples(bb[0],bbL,chunk,0);
      blip_read_samples(bb[1],bbR,chunk,0);
      for (int i=0; i<chunk; i++) { outBuf[i*2]=bbL[i]; outBuf[i*2+1]=bbR[i]; }
      size_t written=fwrite(outBuf,1,chunk*4,dump);
      dumpDataSize+=written;
      pos+=chunk;
    }
    wavFinalizeHeader(dump,dumpDataSize);
    fclose(dump);
    printf("wrote tfmx.wav (%ld bytes audio data)\n",dumpDataSize);
    printf("\n--- Per-Channel RMS (raw sample*vol) ---\n");
    for (int ch=0; ch<4; ch++) {
      double rms = (p.chanSampleCount > 0) ? sqrt(p.chanSumSq[ch] / p.chanSampleCount) : 0;
      const char* side = ((ch&1)^((ch&2)>>1)) ? "R" : "L";
      printf("  Ch %d (%s): RMS = %.1f\n", ch, side, rms);
    }
    printf("---\n");
    return 0;
  }

#ifdef _SYNC_VBLANK
  if (syncVBlank) {
    syncfd=open("/dev/dri/card0",O_RDWR);
    if (syncfd<0) {
      perror("cannot open sync");
      return 1;
    }
    drm_version ver;
    /*if (ioctl(syncfd,DRM_IOCTL_VERSION,&ver)<0) {
      printf("could not get version.\n");
      return 1;
    }*/
  }
#endif

  printVersion();
  printf("opening audio\n");
  
  if (SDL_Init(SDL_INIT_AUDIO)<0) {
    printf("SDL_Init failed: %s\n",SDL_GetError());
    return 1;
  }

  ac.freq=44100;
  ac.format=AUDIO_S16;
  ac.channels=2;
  ac.samples=1024;
  if (hle) {
    ac.callback=processHLE;
  } else {
    ac.callback=process;
  }
  ac.userdata=NULL;
  ai=SDL_OpenAudioDevice(NULL,0,&ac,&ar,SDL_AUDIO_ALLOW_CHANNELS_CHANGE|SDL_AUDIO_ALLOW_FREQUENCY_CHANGE);
  if (ai==0) {
    printf("SDL_OpenAudioDevice failed: %s\n",SDL_GetError());
    SDL_Quit();
    return 1;
  }
  sr=ar.freq;

  if (dumpFile) {
    dump=fopen("tfmx.wav","wb");
    if (dump==NULL) {
      perror("cannot dump");
      return 1;
    }
    dumpDataSize=0;
    wavWriteHeader(dump,ar.freq,ar.channels,16);
  }
  if (ntsc) {
    targetSR=3579545;
    speed=59659;
  } else {
    targetSR=3546895;
    speed=70937;
  }
  if (defCIAVal) {
    p.setCIAVal(defCIAVal);
  } else {
    p.setCIAVal(speed);
  }

  bb[0]=blip_new(32768);
  bb[1]=blip_new(32768);
  blip_set_rates(bb[0],targetSR,sr);
  blip_set_rates(bb[1],targetSR,sr);

  p.hleRate=float((double)targetSR/(double)sr);

  printf("running.\n");
  p.play(songid);
  SDL_PauseAudioDevice(ai,0);
  
#ifdef _WIN32
  SetConsoleCtrlHandler(handleConsoleCtrl,TRUE);
#else
  sigemptyset(&intsa.sa_mask);
  intsa.sa_flags=0;
  intsa.sa_handler=handleTerm;
  sigaction(SIGINT,&intsa,NULL);
#endif

  setvbuf(stdin,NULL,_IONBF,1);

#ifdef _WIN32
  HANDLE winin=GetStdHandle(STD_INPUT_HANDLE);
  HANDLE winout=GetStdHandle(STD_OUTPUT_HANDLE);
  int termprop=0;
  int termpropi=0;
  GetConsoleMode(winout,(LPDWORD)&termprop);
  GetConsoleMode(winin,(LPDWORD)&termpropi);
  termprop|=ENABLE_VIRTUAL_TERMINAL_PROCESSING;
  termpropi&=~ENABLE_LINE_INPUT;
  termpropi&=~ENABLE_PROCESSED_INPUT;
  SetConsoleMode(winout,termprop);
  SetConsoleMode(winin,termpropi);
#else
  if (tcgetattr(0,&termprop)!=0) {
    return 1;
  }
  memcpy(&termpropold,&termprop,sizeof(struct termios));
  termprop.c_lflag&=~ECHO;
  termprop.c_lflag&=~ICANON;
  if (tcsetattr(0,TCSAFLUSH,&termprop)!=0) {
    return 1;
  }
#endif
  
  //p.lock(0,2000000);
  //p.lock(1,2000000);
  //p.lock(3,2000000);

#ifdef _SYNC_VBLANK
  struct timespec vt1, vt2;
  drm_wait_vblank_t vblank;
  vt1=curTime(CLOCK_MONOTONIC);
  if (syncVBlank) while (!quit) {
    vblank.request.sequence=1;
    vblank.request.type=_DRM_VBLANK_RELATIVE;
    if (ioctl(syncfd,DRM_IOCTL_WAIT_VBLANK,&vblank)<0) continue;
    vt2=curTime(CLOCK_MONOTONIC);
    if ((vt2-vt1).tv_nsec>1000000) {
      p.setCIAVal(targetSR*((vt2-vt1).tv_nsec/1000)/1000000);
    }
    vt1=vt2;
  } else
#endif
  
  while (!quit) {
    int c;
    c=fgetc(stdin);
    if (c==EOF) break;
    switch (c) {
      case 3:
        handleTerm(0);
        break;
      case '\t':
        songid=(songid+1)%32;
        p.play(songid);
        printf("subsong %d\n",songid);
        break;
      case 27:
        if (fgetc(stdin)=='[' && fgetc(stdin)=='Z') {
          songid=(songid+31)%32;
          p.play(songid);
          printf("subsong %d\n",songid);
        }
        break;
      case '\n':
      case '\r':
        p.trace=!p.trace;
        printf("frame trace: %s\n",truth[p.trace]);
        break;
      case '\\':
        speed=70937*6;
        p.setCIAVal(speed);
        printf("CIA value: %d (%.2fHz)\n",speed,(double)targetSR/(double)speed);
        break;
      case '~':
        p.traceS=!p.traceS;
        printf("register trace: %s\n",truth[p.traceS]);
        break;
      case '\b': case 127:
        if (ntsc) {
          speed=59659;
        } else {
          speed=70937;
        }
        printf("CIA value: %d (%.2fHz)\n",speed,(double)targetSR/(double)speed);
        p.setCIAVal(speed);
        break;
      case '[':
        speed+=speed/11;
        printf("CIA value: %d (%.2fHz)\n",speed,(double)targetSR/(double)speed);
        p.setCIAVal(speed);
        break;
      case ']':
        speed-=speed/11;
        printf("CIA value: %d (%.2fHz)\n",speed,(double)targetSR/(double)speed);
        p.setCIAVal(speed);
        break;
      case '{':
        speed<<=1;
        printf("CIA value: %d (%.2fHz)\n",speed,(double)targetSR/(double)speed);
        p.setCIAVal(speed);
        break;
      case '}':
        speed>>=1;
        printf("CIA value: %d (%.2fHz)\n",speed,(double)targetSR/(double)speed);
        p.setCIAVal(speed);
        break;
      case '`':
        ntsc=!ntsc;
        printf("TV standard: %s\n",ntsc?("NTSC"):("PAL"));
        if (ntsc) {
          targetSR=3579545;
          speed=59659;
        } else {
          targetSR=3546895;
          speed=70937;
        }
        p.setCIAVal(speed);
        p.hleRate=float((double)targetSR/(double)sr);
        printf("CIA value: %d (%.2fHz)\n",speed,(double)targetSR/(double)speed);
        break;
      case '1':
        printf("channel 0 mute: %s\n",truth[p.mute(0)]);
        break;
      case '2':
        printf("channel 1 mute: %s\n",truth[p.mute(1)]);
        break;
      case '3':
        printf("channel 2 mute: %s\n",truth[p.mute(2)]);
        break;
      case '4':
        printf("channel 3 mute: %s\n",truth[p.mute(3)]);
        break;
      case '5':
        p.traceC[0]=!p.traceC[0];
        printf("channel 0 macro trace: %s\n",truth[p.traceC[0]]);
        break;
      case '6':
        p.traceC[1]=!p.traceC[1];
        printf("channel 1 macro trace: %s\n",truth[p.traceC[1]]);
        break;
      case '7':
        p.traceC[2]=!p.traceC[2];
        printf("channel 2 macro trace: %s\n",truth[p.traceC[2]]);
        break;
      case '8':
        p.traceC[3]=!p.traceC[3];
        printf("channel 3 macro trace: %s\n",truth[p.traceC[3]]);
        break;
      default:
        if (c>='A') {
          p.lock(3,32);
          p.playMacro(c-'A',20,15,3,0,0);
        }
        break;
    }
  }

  SDL_CloseAudioDevice(ai);

  if (dumpFile) {
    printf("closing dump\n");
    fflush(dump);
    wavFinalizeHeader(dump,dumpDataSize);
    fclose(dump);
    dumpFile=false;
  }
  
  printf("quit!\n");
  finish();
  return 0;
}
