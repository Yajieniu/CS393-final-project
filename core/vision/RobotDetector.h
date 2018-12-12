#pragma once

#include <vision/ObjectDetector.h>
#include <vision/structures/RobotCandidate.h>


class TextLogger;
class CV_EXPORTS_W SURF;

/// @ingroup vision
class RobotDetector : public ObjectDetector {
 public:
  RobotDetector(DETECTOR_DECLARE_ARGS);
  void init(TextLogger* tl){ textlogger = tl; }
  unsigned char* getSegImg();
  std::vector<RobotCandidate*> findRobots(vector<Blob> &blobs, 
	std::vector<RobotCandidate*> robot_candidates);
  uint16_t updateCenterY(unsigned char col_height[]);
  vector<Blob> filterRoblobs(vector<Blob> &blobs, int size);
  void filterWallAndLine(Blob &whiteBlob);
  void pickSURFArea(Blob &blob);

 private:
  TextLogger* textlogger;

  // vec/tor<RobotCandidate*> robot_candidates;

};
