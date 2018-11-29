#pragma once

#include <vision/ObjectDetector.h>

class TextLogger;

/// @ingroup vision
class RobotDetector : public ObjectDetector {
 public:
  RobotDetector(DETECTOR_DECLARE_ARGS);
  void init(TextLogger* tl){ textlogger = tl; }
  unsigned char* getSegImg();
  void findRobots(vector<Blob> &blobs);
 private:
  TextLogger* textlogger;
};
