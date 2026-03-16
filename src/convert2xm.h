#ifndef CONVERT2XM_H
#define CONVERT2XM_H

/* Convert TFMX mdat+smpl to XM file. Returns true on success. */
bool convertToXM(const char* mdatPath, const char* smplPath, const char* outPath, int subsong);

#endif
