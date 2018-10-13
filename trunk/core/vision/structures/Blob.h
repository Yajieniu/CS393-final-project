#ifndef BLOB_H
#define BLOB_H

#include <vision/VisionConstants.h>
#include <vision/enums/Colors.h>
#include <vector>
#include <inttypes.h>

/// @ingroup vision
struct Blob {
  uint16_t xi, xf, dx, yi, yf, dy;
  // added for assignment 1
  Color color;
  uint16_t lpCount;
  std::vector<uint32_t> lpIndex;
  float diffStart;
  float diffEnd;
  float doubleDiff;
  uint16_t widthStart;
  uint16_t widthEnd;
  uint16_t avgX;
  uint16_t avgY;
  float avgWidth;
  float correctPixelRatio;
  bool invalid;

  // GOAL DETECTION
  int edgeSize;
  int edgeStrength;

  Blob() : lpIndex(MAX_BLOB_VISIONPOINTS, 0) { }
};

inline ostream& operator<<(ostream &Str, Blob const& v) {
  Str << "(" << v.avgX << ", " << v.avgY << ") ";
  Str << "(" << v.xi << ", " << v.xf << ") ";
  Str << "(" << v.yi << ", " << v.yf << ") ";

  return Str;
}

/// @ingroup vision
bool sortBlobAreaPredicate(Blob* left, Blob* right);
bool BlobCompare(Blob a, Blob b);
vector<Blob> filterBlobs(vector<Blob> &blobs, Color color, int size);
double calculateBlobAspectRatio(const Blob &b);
double calculateBlobArea(const Blob &b);

#endif
