#ifndef CONVERT2XM_H
#define CONVERT2XM_H

enum PanPreset {
  PAN_SOFT=0,
  PAN_AMIGA,
  PAN_HEADPHONE
};

/* Convert TFMX mdat+smpl to XM file. Returns true on success. */
bool convertToXM(const char* mdatPath, const char* smplPath, const char* outPath, int subsong, PanPreset pan=PAN_SOFT);

#endif
