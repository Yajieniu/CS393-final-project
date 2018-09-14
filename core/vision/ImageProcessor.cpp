#include <vision/ImageProcessor.h>
#include <vision/Classifier.h>
#include <vision/BeaconDetector.h>
#include <vision/Logging.h>
#include <iostream>

#define STEP 1
// #define BLOB_THRESHOLD 200/STEP
// #define GOAL_THRESHOLD 1500 / STEP

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
  // if(camera_ == Camera::BOTTOM) return;
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


void ImageProcessor::markBall(int imageX, int imageY, int radius) {

  WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];

  // std::cout << "Ball " << imageX << " " << imageY << " " << ( camera_ == Camera::TOP ) << std::endl;


  ball->imageCenterX = imageX;
  ball->imageCenterY = imageY;

  Position p = cmatrix_.getWorldPosition(imageX, imageY);
  ball->visionBearing = cmatrix_.bearing(p);
  ball->visionElevation = cmatrix_.elevation(p);
  ball->visionDistance = cmatrix_.groundDistance(p);

  ball->fromTopCamera = camera_ == Camera::TOP;

  ball->radius = radius;

  // std::cout << "Ball bearing = " << ball->visionBearing << endl;
  // std::cout << "Ball elevation = " << ball->visionElevation << endl;
  // std::cout << "Ball distance = " << ball->visionDistance << endl;
  getBall();
  ball->seen = true;
}

void ImageProcessor:: markGoal(int imageX, int imageY) {

  WorldObject* goal = &vblocks_.world_object->objects_[WO_OWN_GOAL];

  goal->imageCenterX = imageX;
  goal->imageCenterY = imageY;

  // goal->imageCenterX = 0;
  // goal->imageCenterY = 0;

  Position p = cmatrix_.getWorldPosition(imageX, imageY);
  goal->visionBearing = cmatrix_.bearing(p);
  goal->visionElevation = cmatrix_.elevation(p);
  goal->visionDistance = cmatrix_.groundDistance(p);

  goal->seen = true;
}

void ImageProcessor:: markBeacon(WorldObjectType beacon_name, int beaconX, int beaconY) {

  static map<WorldObjectType,int> heights = {
    { WO_BEACON_BLUE_YELLOW, 300 },
    { WO_BEACON_YELLOW_BLUE, 300 },
    { WO_BEACON_BLUE_PINK, 200 },
    { WO_BEACON_PINK_BLUE, 200 },
    { WO_BEACON_PINK_YELLOW, 200 },
    { WO_BEACON_YELLOW_PINK, 200 }
  };

  WorldObject* beacon = &vblocks_.world_object->objects_[beacon_name];

  beacon->imageCenterX = beaconX;
  beacon->imageCenterY = beaconY;

  Position p = cmatrix_.getWorldPosition(beaconX, beaconY, heights[beacon_name]);
  beacon->visionBearing = cmatrix_.bearing(p);
  beacon->visionElevation = cmatrix_.elevation(p);
  beacon->visionDistance = cmatrix_.groundDistance(p);

  beacon->seen = true;
}

bool ImageProcessor::generalBlobFilter(block_t* block) {
  if (block->parent != block) {
    return false;
  }

  return true;
}

bool ImageProcessor::lookLikeBall(block_t* block) {
  if (block->parent != block || block->color != c_ORANGE) {
    return false;
  }

  // int centerX = block->meanX;
  // int centerY = block->meanY;
  // int radius = (block->maxY - block->minY + block->maxX - block->minX) / 4;

  // int step = 30;
  // int count = 0;
  // for (int i = 0; i < 360; i+=step) {
  //   int x = centerX + cos(step);
  //   int y = centerY + sin()
  // }

  // // std::cout << block->x << " " << block->y << " " << static_cast<int> (block->color) << std::endl;
  int width =  block->maxX - block->minX;
  int height = block->maxY - block->minY;

  if (width >= 1.5 * height || height >= 1.5 * width) {
    return false;
  }



  int radius = (width+height) / 4;
  if (radius * radius >= block->count / 2.8) { return false; }
  if (radius * radius <= block->count / 3.4) { return false; }

  return true;
}
    
bool ImageProcessor::lookLikeGoal(block_t* block) {

  return true;
}

bool ImageProcessor::lookLikeBeacon(block_t* block, 
  WorldObjectType beacon_name, int& count, int& meanX, int& meanY) {
  
  if (!generalBlobFilter(block)) {
    return false;
  }

  static map<WorldObjectType, pair<Color,Color> > beacon_colors = {
    { WO_BEACON_BLUE_YELLOW, {c_BLUE, c_YELLOW} },
    { WO_BEACON_YELLOW_BLUE, {c_YELLOW, c_BLUE} },
    { WO_BEACON_BLUE_PINK, {c_BLUE, c_PINK} },
    { WO_BEACON_PINK_BLUE, {c_PINK, c_BLUE} },
    { WO_BEACON_PINK_YELLOW, {c_PINK, c_YELLOW} },
    { WO_BEACON_YELLOW_PINK, {c_YELLOW, c_PINK} }
  };

  static map<WorldObjectType,int> beacon_types = {
    { WO_BEACON_BLUE_YELLOW, 2 },
    { WO_BEACON_YELLOW_BLUE, 2 },
    { WO_BEACON_BLUE_PINK, 1 },
    { WO_BEACON_PINK_BLUE, 1 },
    { WO_BEACON_PINK_YELLOW, 1 },
    { WO_BEACON_YELLOW_PINK, 1 }
  };

  c1 = beacon_colors[beacon_name].first;
  c2 = beacon_colors[beacon_name].second;
  type = beacon_types[beacon_name];

  if (block->color != c1) {
    return false;
  }



  return true;

}


void ImageProcessor::detectBlob() {

  if (camera_ == Camera::BOTTOM) { return; }

  int size = iparams_.width/STEP * iparams_.height/STEP;
  block_t* blocks = new block_t[size];
  RLE(blocks);

  // Merge blocks
  for (int y = 1; y < iparams_.height/STEP; y++) {
    block_t* topRow = &blocks[(y-1)*iparams_.width/STEP];
    block_t* bottomRow = &blocks[y*iparams_.width/STEP];
    mergeRow(topRow, bottomRow);
  }

  // some variables to help detect objects
  // ball
  int largestBallSize = 0;
  int ballRadius = 0;
  int ballX = 0;
  int ballY = 0;

  // goal
  int largestGoalSize = 0;
  int goalX = 0;
  int goalY = 0;

  // beacon
  const int n_beacons = 6;
  WorldObjectType beacons[n_beacons] = {WO_BEACON_BLUE_YELLOW,
                                        WO_BEACON_YELLOW_BLUE,
                                        WO_BEACON_BLUE_PINK,
                                        WO_BEACON_PINK_BLUE,
                                        WO_BEACON_PINK_YELLOW,
                                        WO_BEACON_YELLOW_PINK};
  int largestBeaconSize[n_beacons] = {0,0,0,0,0,0};
  int beaconX[n_beacons] = {0,0,0,0,0,0};
  int beaconY[n_beacons] = {0,0,0,0,0,0};


  // detect objects
  for (int y = 0; y < iparams_.height/STEP; y++) {
    for (int x = 0; x < iparams_.width/STEP;) {
      int index = y * iparams_.width/STEP + x;
      auto block = &blocks[index];
      if (lookLikeBall(block) && block->count > largestBallSize) {
        largestBallSize = block->count;
        ballX = block->meanX * iparams_.width;
        ballY = block->meanY * iparams_.height;
        ballRadius = (block->maxX - block->minX + block->maxY - block->minY) / 4;
      }

      if (lookLikeGoal(block) && block->count > largestGoalSize) {
        largestGoalSize = block->count;
        goalX = block->meanX * iparams_.width;
        goalY = block->meanY * iparams_.height;
      }

      for (int i_beacon = 0; i_beacon < n_beacons; ++i_beacon) {
        int count; double meanX, meanY;
        if (lookLikeBeacon(block, beacons[i_beacon], &count, &meanX, &meanY) 
            && block->count > largestBeaconSize[i_beacon]) {
          largestBeaconSize[i_beacon] = count;
          beaconX[i_beacon] = meanX * iparams_.width;
          beaconY[i_beacon] = meanY * iparams_.height;
        }
      }

      x += block->length;
    }
  }

  if (largestBallSize > 0) {
    markBall(ballX , ballY, ballRadius);
    // std::cout << "Ball " << ballX << " " << ballY << " " << largestBallSize << std::endl;
  }

  if (largestGoalSize > 0) {
    markGoal(goalX, goalY);
    // std::cout << "Goal " << goalX << " " << goalY << " " << largestGoalSize << std::endl;
  }

  for (int i_beacon = 0; i_beacon < n_beacons; ++i_beacon) {
    if (largestBeaconSize[i_beacon] > 0) {
      markBeacon(beacons[i_beacon], beaconX[i_beacon], beaconY[i_beacon]);
    }
  }

  delete[] blocks;


}

void ImageProcessor::mergeRow(block_t *rowA, block_t* rowB) {
  int indexA = 0;
  int indexB = 0;

  while (indexA < iparams_.width / STEP || indexB < iparams_.width / STEP) {
    auto blockA = &rowA[indexA];
    auto blockB = &rowB[indexB];
    
    mergeBlock(blockA, blockB);

    if (indexB >= iparams_.width / STEP || indexA + blockA->length <= indexB + blockB->length) {
      indexA += blockA->length;
    } else {
      indexB += blockB->length;
    }
  }
}

void ImageProcessor::mergeBlock(block_t* blockA, block_t* blockB) {
  if (blockA->color != blockB->color) { return; }

  if (blockA->x + blockA->length > blockB->x && blockB->x + blockB->length > blockA->length) {
    unionBlock(blockA, blockB);
  }
}

void ImageProcessor::initBlock(block_t* blocks, int x, int y, int length, unsigned char color) {
  int blockIndex = y/STEP * iparams_.width/STEP + x/STEP;
  auto block = &blocks[blockIndex-length];

  block->parent = block;

  block->length = length;
  block->color = color;

  block->x = x-length;
  block->y = y;

  block->minX = block->x;
  block->maxX = block->x + length;
  block->minY = block->y;
  block->maxY = block->y;


  block->meanX = (block->x + length/2.) / iparams_.width;
  block->meanY = y*1. / iparams_.height;
  block->count = length;

  for (int i = blockIndex-length+1; i < blockIndex; ++i) {
    blocks[i] = block;
  }

  // DEBUG
  // if (color == c_BLUE) {
  //   std::cout << "Blue " << x << ", " << y << " " << length << std::endl;
  // }

  // if (color == c_ORANGE) {
  //   std::cout << "Orange " << x << ", " << y << " " << length << std::endl;
  // }

  // if (color == c_FIELD_GREEN) {
  //   std::cout << "Green " << x << ", " << y << " " << length << std::endl;
  // }
}

void ImageProcessor::RLE(block_t* blocks) {
  auto colors = getSegImg();
  for (int y = 0; y < iparams_.height; y+=STEP) {
    int length = 1;
    auto color = colors[y * iparams_.width];

    int x;
    for (x = STEP; x < iparams_.width; x+=STEP) {
      int colorIndex = y * iparams_.width + x;
      auto currentColor = colors[colorIndex];
      if (currentColor != color) {
        initBlock(blocks, x, y, length, color);
        length = 1;
        color = currentColor;
      } else {
        length++;
      }
    }

    initBlock(blocks, x, y, length, color);

  }
}

block_t* ImageProcessor::findBlockParent(block_t* block) {
  if (block->parent != block) {
    return block->parent = findBlockParent(block->parent);
  }
  return block->parent;
}

void ImageProcessor::unionBlock(block_t* blockA, block_t* blockB) {
  auto parentA = findBlockParent(blockA);
  auto parentB = findBlockParent(blockB);

  // std::cout << "Merging [" << blockB->x <<" " << blockB->y <<"] to [" << blockA->x << " " << blockA->y << "]!" << std::endl;

  // if (parentA == parentB) {
  //   // std::cout << parentA->x << " " << parentA->y << std::endl;
  //   std::cout << "Error!" << std::endl;
  // }

  // If share same parent already, then 
  if (parentA == parentB) {
    return;
    // parentB = blockB;
  }

  // Always merge to the top-left block
  // if (parentB->x < parentA->x || parentB->y < parentA->y) {
  //   auto tmp = parentB;
  //   parentB = parentA;
  //   parentA = tmp;
  // }

  long totalCount = parentA->count + parentB->count;
  double weightA = parentA->count*1. / totalCount;
  double weightB = parentB->count*1. / totalCount;
  parentA->meanX = parentA->meanX * weightA + parentB->meanX * weightB;
  parentA->meanY = parentA->meanY * weightA + parentB->meanY * weightB;

  parentA->minX = MIN(parentA->minX, parentB->minX);
  parentA->minY = MIN(parentA->minY, parentB->minY);
  parentA->maxX = MAX(parentA->maxX, parentB->maxX);
  parentA->maxY = MAX(parentA->maxY, parentB->maxY);

  parentA->count += parentB->count;

  parentB->parent = parentA;
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
  // auto ball = ;
  return NULL;
}

WorldObject* ImageProcessor::getBall() {
  auto ball = &vblocks_.world_object->objects_[WO_BALL];
  // std::cout << ball->imageCenterX << " " << ball->imageCenterY << std::endl;
  return &vblocks_.world_object->objects_[WO_BALL];
}
 
void ImageProcessor::enableCalibration(bool value) {
  enableCalibration_ = value;
}

bool ImageProcessor::isImageLoaded() {
  return vblocks_.image->isLoaded();
}
