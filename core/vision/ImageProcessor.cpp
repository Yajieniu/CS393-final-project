#include <vision/ImageProcessor.h>
#include <vision/Classifier.h>
#include <vision/BeaconDetector.h>
#include <vision/Logging.h>
#include <iostream>
#define BLOB_THRESHOLD 50
#define GOAL_THRESHOLD 500

ImageProcessor::ImageProcessor(VisionBlocks& vblocks, const ImageParams& iparams, Camera::Type camera) :
  vblocks_(vblocks), iparams_(iparams), camera_(camera), cmatrix_(iparams_, camera)
{
  enableCalibration_ = false;
  color_segmenter_ = std::make_unique<Classifier>(vblocks_, vparams_, iparams_, camera_);
  beacon_detector_ = std::make_unique<BeaconDetector>(DETECTOR_PASS_ARGS);
  calibration_ = std::make_unique<RobotCalibration>();
}

ImageProcessor::~ImageProcessor() {
}

void ImageProcessor::init(TextLogger* tl){
  textlogger = tl;
  vparams_.init();
  color_segmenter_->init(tl);
  beacon_detector_->init(tl);
}

unsigned char* ImageProcessor::getImg() {
  if(camera_ == Camera::TOP)
    return vblocks_.image->getImgTop();
  return vblocks_.image->getImgBottom();
}

//void ImageProcessor::saveImg(std::string filepath) {
//  cv::Mat mat;
//  int xstep_ = 1 << iparams_.defaultHorizontalStepScale;
//  int ystep_ = 1 << iparams_.defaultVerticalStepScale;
//  cv::resize(color_segmenter_->img_grayscale(), mat, cv::Size(), 1.0 / xstep_, 1.0 / ystep_, cv::INTER_NEAREST); 
  
//  cv::imwrite(filepath, mat);
//}

unsigned char* ImageProcessor::getSegImg(){
  if(camera_ == Camera::TOP)
    return vblocks_.robot_vision->getSegImgTop();
  return vblocks_.robot_vision->getSegImgBottom();
}

unsigned char* ImageProcessor::getColorTable(){
  return color_table_;
}

const CameraMatrix& ImageProcessor::getCameraMatrix(){
  return cmatrix_;
}

void ImageProcessor::updateTransform(){
  BodyPart::Part camera;
  if(camera_ == Camera::TOP)
    camera = BodyPart::top_camera;
  else
    camera = BodyPart::bottom_camera;

  Pose3D pcamera;
  if(enableCalibration_) {
    float joints[NUM_JOINTS], sensors[NUM_SENSORS], dimensions[RobotDimensions::NUM_DIMENSIONS];
    memcpy(joints, vblocks_.joint->values_.data(), NUM_JOINTS * sizeof(float));
    memcpy(sensors, vblocks_.sensor->values_.data(), NUM_SENSORS * sizeof(float));
    memcpy(dimensions, vblocks_.robot_info->dimensions_.values_, RobotDimensions::NUM_DIMENSIONS * sizeof(float));
    Pose3D *rel_parts = vblocks_.body_model->rel_parts_.data(), *abs_parts = vblocks_.body_model->abs_parts_.data();
    calibration_->applyJoints(joints);
    calibration_->applySensors(sensors);
    calibration_->applyDimensions(dimensions);
    ForwardKinematics::calculateRelativePose(joints, rel_parts, dimensions);
#ifdef TOOL
    Pose3D base = ForwardKinematics::calculateVirtualBase(calibration_->useLeft, rel_parts);
    ForwardKinematics::calculateAbsolutePose(base, rel_parts, abs_parts);
#else
    ForwardKinematics::calculateAbsolutePose(sensors, rel_parts, abs_parts);
#endif
    cmatrix_.setCalibration(*calibration_);
    pcamera = abs_parts[camera];
  }
  else pcamera = vblocks_.body_model->abs_parts_[camera];

  if(vblocks_.robot_state->WO_SELF == WO_TEAM_COACH) {
    auto self = vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF];
    pcamera.translation.z += self.height;
  }

  cmatrix_.updateCameraPose(pcamera);
}

bool ImageProcessor::isRawImageLoaded() {
  if(camera_ == Camera::TOP)
    return vblocks_.image->isLoaded();
  return vblocks_.image->isLoaded();
}

int ImageProcessor::getImageHeight() {
  return iparams_.height;
}

int ImageProcessor::getImageWidth() {
  return iparams_.width;
}

double ImageProcessor::getCurrentTime() {
  return vblocks_.frame_info->seconds_since_start;
}

void ImageProcessor::setCalibration(const RobotCalibration& calibration){
  *calibration_ = calibration;
}

void ImageProcessor::processFrame(){
  if(camera_ == Camera::BOTTOM) return;
  tlog(30, "Process Frame camera %i", camera_);
  
  // Horizon calculation
  tlog(30, "Calculating horizon line");
  updateTransform();
  HorizonLine horizon = HorizonLine::generate(iparams_, cmatrix_, 20000);
  vblocks_.robot_vision->horizon = horizon;
  tlog(30, "Classifying Image: %i", camera_);
  if(!color_segmenter_->classifyImage(color_table_)) return;
  detectBlob();
  beacon_detector_->findBeacons();
}


void ImageProcessor::markBall(int imageX, int imageY) {

  std::cout << "Detect Ball: " << imageX << ", " << imageY << std::endl;

  WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];

  ball->imageCenterX = imageX;
  ball->imageCenterY = imageY;

  Position p = cmatrix_.getWorldPosition(imageX, imageY);
  ball->visionBearing = cmatrix_.bearing(p);
  ball->visionElevation = cmatrix_.elevation(p);
  ball->visionDistance = cmatrix_.groundDistance(p);

  // std::cout << "Ball bearing = " << ball->visionBearing << endl;
  // std::cout << "Ball elevation = " << ball->visionElevation << endl;
  // std::cout << "Ball distance = " << ball->visionDistance << endl;

  ball->seen = true;
}

void ImageProcessor:: markGoal(int imageX, int imageY, int maxY) {
  std::cout << "Detect Goal: " << imageX << ", " << imageY << std::endl;

  WorldObject* goal = &vblocks_.world_object->objects_[WO_OWN_GOAL];

  goal->imageCenterX = imageX;
  goal->imageCenterY = maxY;

  Position p = cmatrix_.getWorldPosition(imageX, maxY);
  goal->visionBearing = cmatrix_.bearing(p);
  goal->visionElevation = cmatrix_.elevation(p);
  goal->visionDistance = cmatrix_.groundDistance(p);

  std::cout << "goal bearing = " << goal->visionBearing << endl;
  std::cout << "goal elevation = " << goal->visionElevation << endl;
  std::cout << "goal distance = " << goal->visionDistance << endl;

  goal->seen = true;

}

void ImageProcessor::detectBlob() {

  int size = iparams_.height * iparams_.width;
  bool* visited = new bool[size];

  long bestBallCount = 0;
  long bestGoalCount = 0;

  for (int x = 0; x < iparams_.width; x+=2) {
    for (int y = 0; y < iparams_.height; y+=2) {
      auto c = getSegImg()[y * iparams_.width + x];
      long long totalX = 0;
      long long totalY = 0;
      int maxY = 0;
      long count = 0;
      findBlobDFS(x, y, visited, &totalX, &totalY, &maxY, &count, c);
      int meanX = totalX / count;
      int meanY = totalY / count;

      // If it is orange
      if (count > bestBallCount && c == c_ORANGE && count >= BLOB_THRESHOLD) {
        markBall(meanX, meanY);
      }

      if (count > bestGoalCount && c == c_BLUE && count >= GOAL_THRESHOLD) {

        markGoal(meanX, meanY, maxY);
      }
    }
  }
}


void ImageProcessor::findBlobDFS(int x, int y, bool* visited, long long* totalX, long long* totalY, int* maxY, long *count, unsigned char color) {
  visited[y * iparams_.width + x] = true;
  *totalX += x;
  *totalY += y;
  *count += 1;
  *maxY = MAX(*maxY, y);

  // xs->push_back(x);
  // ys->push_back(y);
  auto colors = getSegImg();
  for (int xOffset = 0; xOffset <= 5; xOffset++) {
    for (int yOffset = 0; yOffset <= 5; yOffset++) {
      int newX = x + xOffset;
      int newY = y + yOffset;

      if (newX >= 0 && newX < iparams_.width && 
          newY >= 0 && newY < iparams_.height && 
          !visited[newY * iparams_.width + newX] && 
          colors[newY * iparams_.width + newX] == color
      ) {
        findBlobDFS(newX, newY, visited, totalX, totalY, maxY, count, color);
      }
    }
  }
}


int ImageProcessor::getTeamColor() {
  return vblocks_.robot_state->team_;
}

void ImageProcessor::SetColorTable(unsigned char* table) {
  color_table_ = table;
}

float ImageProcessor::getHeadChange() const {
  if (vblocks_.joint == NULL)
    return 0;
  return vblocks_.joint->getJointDelta(HeadPan);
}

std::vector<BallCandidate*> ImageProcessor::getBallCandidates() {
  return std::vector<BallCandidate*>();
}

BallCandidate* ImageProcessor::getBestBallCandidate() {
  return NULL;
}
 
void ImageProcessor::enableCalibration(bool value) {
  enableCalibration_ = value;
}

bool ImageProcessor::isImageLoaded() {
  return vblocks_.image->isLoaded();
}
