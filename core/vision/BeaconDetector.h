#pragma once

#include <vision/ObjectDetector.h>

#define ASPECT_RATIO_LOW_BOUND 0.5
#define ASPECT_RATIO_HIGH_BOUND 2.0
#define OCCLUDED_ASPECT_RATIO_HIGH_BOUND 0.85
#define DENSITY_LOW_BOUND 0.4

// #define ENABLE_AREA_SIM_FILTERING
#define AREA_SIM_LOW_BOUND 0.4
#define AREA_SIM_HIGH_BOUND 2.0

#define WHITE_BELOW_BEACON_LOW_BOUND 0.5
#define COLOR_ABOVE_BEACON_HIGH_BOUND 0.25
#define VERTICAL_SEPARATION_HIGH_BOUND 10

class TextLogger;

/// @ingroup vision
class BeaconDetector : public ObjectDetector {
 public:
  BeaconDetector(DETECTOR_DECLARE_ARGS);
  void init(TextLogger* tl){ textlogger = tl; }
  unsigned char* getSegImg();
  bool validateInverted(pair<Blob, Blob> &bblob);
  bool validateUp(pair<Blob, Blob> &bblob);
  pair<Blob, Blob> findBeaconsOfType(const vector<Blob> &tb, const vector<Blob> &bb);
  void findBeacons(vector<Blob> &blobs);
 private:
  TextLogger* textlogger;
};
