#include <vision/ImageProcessor.h>
#include <vision/Classifier.h>
#include <vision/BeaconDetector.h>
#include <vision/Logging.h>
#include <iostream>
#include <cmath>

vector<RLE*> ImageProcessor::getRLERow(int y, int width, int &start_idx) {
    // handle NULL case
    int xstep = 1 << iparams_.defaultHorizontalStepScale;
    int ystep = 1 << iparams_.defaultVerticalStepScale;
    auto prev_color = getSegImg()[y * width];
    auto prev_idx = 0;
    vector<RLE*> encoding;
    for(int x = 0; x < width; x += xstep) {
        auto c = getSegImg()[y * width + x];
        if(c == prev_color)
            continue;
        else {
            encoding.push_back(new RLE(y, prev_idx, x - 1, start_idx, prev_color, ystep));
            start_idx++;
            prev_color = c;
            prev_idx = x;
        }
    }
    encoding.push_back(new RLE(y, prev_idx, width - 1, start_idx, prev_color, ystep));
    start_idx++;

    return encoding;
}

int ImageProcessor::getParent(int idx) {
    if(rle_ptr.find(idx) == rle_ptr.end())
        return -1;
    if(rle_ptr[idx]->parent == rle_ptr[idx]->curr)
        return rle_ptr[idx]->parent;
    // Path compression
    int p = getParent(rle_ptr[idx]->parent);
    rle_ptr[idx]->parent = p;
    return p;
}

void ImageProcessor::mergeBlobs(int idx1, int idx2) {
    int p1 = getParent(idx1);
    int p2 = getParent(idx2);
    if(p1 == -1 || p2 == -1) {
        std::cout << "Unknown RLE" << endl;
        return;
    }
    if(p1 == p2)
        return;
    // Union by rank
    int r1 = rle_ptr[p1]->rank;
    int r2 = rle_ptr[p2]->rank;
    if(r1 > r2) {
        rle_ptr[p2]->parent = p1;
        rle_ptr[p1]->npixels += rle_ptr[p2]->npixels;
        rle_ptr[p1]->xi = min(rle_ptr[p1]->xi, rle_ptr[p2]->xi);
        rle_ptr[p1]->xf = max(rle_ptr[p1]->xf, rle_ptr[p2]->xf);
        rle_ptr[p1]->yi = min(rle_ptr[p1]->yi, rle_ptr[p2]->yi);
        rle_ptr[p1]->yf = max(rle_ptr[p1]->yf, rle_ptr[p2]->yf);
        rle_ptr[p1]->xsum += rle_ptr[p2]->xsum;
        rle_ptr[p1]->ysum += rle_ptr[p2]->ysum;
    }
    else if(r2 > r1) {
        rle_ptr[p1]->parent = p2;
        rle_ptr[p2]->npixels += rle_ptr[p1]->npixels;
        rle_ptr[p2]->xi = min(rle_ptr[p1]->xi, rle_ptr[p2]->xi);
        rle_ptr[p2]->xf = max(rle_ptr[p1]->xf, rle_ptr[p2]->xf);
        rle_ptr[p2]->yi = min(rle_ptr[p1]->yi, rle_ptr[p2]->yi);
        rle_ptr[p2]->yf = max(rle_ptr[p1]->yf, rle_ptr[p2]->yf);
        rle_ptr[p2]->xsum += rle_ptr[p1]->xsum;
        rle_ptr[p2]->ysum += rle_ptr[p1]->ysum;
    }
    else {
        rle_ptr[p2]->parent = p1;
        rle_ptr[p1]->rank++;
        rle_ptr[p1]->npixels += rle_ptr[p2]->npixels;
        rle_ptr[p1]->xi = min(rle_ptr[p1]->xi, rle_ptr[p2]->xi);
        rle_ptr[p1]->xf = max(rle_ptr[p1]->xf, rle_ptr[p2]->xf);
        rle_ptr[p1]->yi = min(rle_ptr[p1]->yi, rle_ptr[p2]->yi);
        rle_ptr[p1]->yf = max(rle_ptr[p1]->yf, rle_ptr[p2]->yf);
        rle_ptr[p1]->xsum += rle_ptr[p2]->xsum;
        rle_ptr[p1]->ysum += rle_ptr[p2]->ysum;
    }
}

void ImageProcessor::mergeEncodings(vector<RLE*> &prev_encoding, vector<RLE*> &encoding) {
    if(prev_encoding.size() == 0 || encoding.size() == 0)
        return;
    int i = 0, j = 0;
    while(i < prev_encoding.size() && j < encoding.size()) {
        if(prev_encoding[i]->rcol < encoding[j]->lcol) {
            i++;
        }
        else if(prev_encoding[i]->lcol > encoding[j]->rcol) {
            j++;
        }
        else {
            // overlap detected, if colors match then merge blobs
            if(prev_encoding[i]->color == encoding[j]->color) {
                mergeBlobs(prev_encoding[i]->curr, encoding[j]->curr);
            }
            // progress pointers
            if(prev_encoding[i]->rcol >= encoding[j]->rcol) {
                j++;
            }
            else {
                i++;
            }
        }
    }
}

Blob makeBlob(RLE* r) {
    Blob b;
    b.xi = r->xi;
    b.xf = r->xf;
    b.yi = r->yi;
    b.yf = r->yf;
    b.dx = r->xf - r->xi + 1;
    b.dy = r->yf - r->yi + 1;
    b.color = static_cast<Color>(r->color);
    b.lpCount = r->npixels;
    b.avgX = r->xsum / r->npixels;
    b.avgY = r->ysum / r->npixels;
    
    return b;
}

void ImageProcessor::calculateBlobs() {
    // handle NULL case
    int height = iparams_.height;
    int width = iparams_.width;
    int ystep = 1 << iparams_.defaultVerticalStepScale;
    int loc_idx = 0;
    rle_ptr.clear();
    vector<RLE*> prev_encoding;

    for(int y = 0; y < height; y += ystep) {
        auto encoding = getRLERow(y, width, loc_idx);
        // initialising the hash table with RLE pointers
        for(int i = 0; i < encoding.size(); ++i) {
            assert(rle_ptr.find(encoding[i]->curr) == rle_ptr.end());
            rle_ptr[encoding[i]->curr] = encoding[i];
        }
        mergeEncodings(prev_encoding, encoding);
        prev_encoding = encoding;
    }
    // setting the detected blobs in the vector
    detected_blobs.clear();
    for(auto it = rle_ptr.begin(); it != rle_ptr.end(); ++it) {
        RLE* b = it->second;
        if(b->parent == b->curr) {
            detected_blobs.push_back(makeBlob(b));
        }
        delete(b);
    }
}

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
  // if(vblocks_.robot_state->WO_SELF == WO_TEAM_COACH && camera_ == Camera::BOTTOM) return;

  tlog(30, "Process Frame camera %i", camera_);

  // Horizon calculation
  tlog(30, "Calculating horizon line");
  updateTransform();
  HorizonLine horizon = HorizonLine::generate(iparams_, cmatrix_, 20000);
  vblocks_.robot_vision->horizon = horizon;
  tlog(30, "Classifying Image: %i", camera_);
  if(!color_segmenter_->classifyImage(color_table_)) return;
  calculateBlobs();
  detectBall();
  if(camera_ == Camera::BOTTOM) return;

  detectGoal();
  beacon_detector_->findBeacons(detected_blobs);
}

void ImageProcessor::detectBall() {
    WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];
    if(ball->seen)
        return;

    BallCandidate* ballc = getBestBallCandidate();

    if(ballc == NULL){
        // cout << "Ball not detected" << endl;
        ball->seen = false;
        return;
    }

    ball->imageCenterX = ballc->centerX;
    ball->imageCenterY = ballc->centerY;
    ball->radius = ballc->radius;

    Position p = cmatrix_.getWorldPosition(ballc->centerX, ballc->centerY);
    ball->visionBearing = cmatrix_.bearing(p);
    ball->visionElevation = cmatrix_.elevation(p);
    ball->visionDistance = cmatrix_.groundDistance(p);

    cout << "Ball detected at: " << ballc->centerX << "," << ballc->centerY << endl;
    cout << "Ball pan: " << ball->visionBearing << "   Ball tilt: " << ball->visionElevation << endl;
    cout << "Ball distance: " << ball->visionDistance << endl << endl;

    ball->seen = true;

    if(camera_ == Camera::BOTTOM)
        ball->fromTopCamera = false;
    else
        ball->fromTopCamera = true;
}

void ImageProcessor::findBall(int& imageX, int& imageY) {
}

void ImageProcessor::detectGoal() {
    int imageX = -1, imageY = -1;
    findGoal(imageX, imageY);

    WorldObject* goal = &vblocks_.world_object->objects_[WO_UNKNOWN_GOAL];
    if(imageX == -1 && imageY == -1){
        goal->seen = false;
        return;
    }

    goal->imageCenterX = imageX;
    goal->imageCenterY = imageY;

    Position p = cmatrix_.getWorldPosition(imageX, imageY);
    goal->visionBearing = cmatrix_.bearing(p);
    goal->visionElevation = cmatrix_.elevation(p);
    goal->visionDistance = cmatrix_.groundDistance(p);
    goal->fromTopCamera = (camera_ == Camera::TOP);

    cout << "Goal pan: " << goal->visionBearing << "   Goal tilt: " << goal->visionElevation << endl;
    cout << "Goal distance: " << goal->visionDistance << endl << endl;
    goal->seen = true;
}


void ImageProcessor::findGoal(int& imageX, int& imageY) {
    if(getSegImg() == NULL){
        imageX = -1;
        imageY = -1;
        // cout << "Goal not detected" << endl;
        return;
    }
    auto blueBlobs = filterBlobs(detected_blobs, c_BLUE, 2000);
    sort(blueBlobs.begin(), blueBlobs.end(), BlobCompare);
    if(blueBlobs.size() > 0) {
        // cout << "Goal detected at: " << blueBlobs[0].avgX << "\t" << blueBlobs[0].yf << endl;
        double rectArea = (blueBlobs[0].dx) * (blueBlobs[0].dy);
        double density = (blueBlobs[0].lpCount / rectArea);
        if (density > 0.7) {
            imageX = blueBlobs[0].avgX;
            imageY = blueBlobs[0].yf;
        }
        else {
            // cout << "Skipping " << blueBlobs[0].avgX << " " << blueBlobs[0].yf << " " << density << endl;
            imageX = -1;
            imageY = -1;
        }
    }
    else {
        imageX = -1;
        imageY = -1;
        // cout << "Goal not detected" << endl;
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
    for(int i = 0; i < ball_candidates.size(); ++i) {
        delete(ball_candidates[i]);
    }
    ball_candidates.clear();

    const int width = iparams_.width;
    const int height = iparams_.height;
    const int xstep = (1 << iparams_.defaultHorizontalStepScale);
    const int ystep = (1 << iparams_.defaultVerticalStepScale);
    unsigned char* segImg = getSegImg();

    if(segImg == NULL){
        return ball_candidates;
    }

    vector<Blob> orangeBlobs;

    if (camera_ == Camera::BOTTOM)
        orangeBlobs = filterBlobs(detected_blobs, c_ORANGE, 200);
    else
        orangeBlobs = filterBlobs(detected_blobs, c_ORANGE, 50);

    sort(orangeBlobs.begin(), orangeBlobs.end(), BlobCompare);
    for(int i = 0; i < orangeBlobs.size(); ++i) {

        double sideRatio = double(orangeBlobs[i].dx) / (orangeBlobs[i].dy);
        // cout << sideRatio << endl;
        if (camera_ == Camera::TOP && (sideRatio < 0.6 || sideRatio > 1.4)) {
          // std::cout << "Skipping due to side ratio: " << i << " " << sideRatio << endl;
          // cout << "skipping" << endl;
          continue;
        }

        // area ratio
        double rectArea = (orangeBlobs[i].dx) * (orangeBlobs[i].dy);
        double density = (orangeBlobs[i].lpCount / rectArea);

        if (density < 0.5) {
          // std::cout << "Skipping due to density: " << i << " " << density << endl;
          // cout << "skipping" << endl;
          continue;
        }

        int BALL_MAX_AREA_THRESHOLD = camera_ == Camera::BOTTOM ? iparams_.size : 1600;

        if (rectArea > BALL_MAX_AREA_THRESHOLD)
            continue;

        // filter out candidate if not on green ground
        int xstart = max(orangeBlobs[i].avgX - orangeBlobs[i].dx * 1, 0);
        int xend   = min(orangeBlobs[i].avgX + orangeBlobs[i].dx * 1, width - 1);
        int ystart = min(orangeBlobs[i].avgY + orangeBlobs[i].dy, height - 1);
        int yend   = min(orangeBlobs[i].avgY + orangeBlobs[i].dy * 2, height - 1);
        xstart -= xstart % xstep;
        xend -= xend % xstep;
        ystart -= ystart % ystep;
        yend -= yend % ystep;

        double tot_count = 1;
        double green_count = 0;
        for(int x=xstart; x <= xend; x += xstep){
          for(int y=ystart; y <= yend; y += ystep){
            auto c = static_cast<Color>(segImg[y * width + x]);
            green_count += (c == c_FIELD_GREEN) ? 1 : 0;
            tot_count += 1;
          }
        }
        // If the ball is near bottom of image, it can still be green
        if (green_count / tot_count < 0.2 && (yend - ystart) > 5) {
          continue;
        }

        // std::cout << "Not skipping: " << i << " " << sideRatio << " " << areaRatio << endl;
        // std::cout << "Blob " << i << " " << orangeBlobs[i].avgX << " " << orangeBlobs[i].avgY 
        //       << " " << orangeBlobs[i].lpCount << " " << orangeBlobs[i].dx << " " << orangeBlobs[i].dy << endl;

        BallCandidate* ballc = new BallCandidate();
        ballc->centerX = orangeBlobs[i].avgX;
        ballc->centerY = orangeBlobs[i].avgY;
        ballc->radius = (orangeBlobs[i].dx + orangeBlobs[i].dy) / 4;
        ballc->width = orangeBlobs[i].dx;
        ballc->height = orangeBlobs[i].dy;

        Position p = cmatrix_.getWorldPosition(ballc->centerX, ballc->centerY);
        ballc->groundDistance = cmatrix_.groundDistance(p);
        // [TODO] add the confidence value for ball candidates
        ballc->confidence = 0.0;
        // [TODO] add the blob on the heap and send
        ballc->blob = NULL;
        ballc->valid = true;
        // [TODO] add the absolute and relative positions
        ball_candidates.push_back(ballc);

        break;
    }
    return ball_candidates;
}

BallCandidate* ImageProcessor::getBestBallCandidate() {
    getBallCandidates();
    if(ball_candidates.size() == 0)
        return NULL;
    return ball_candidates[0];
}

void ImageProcessor::enableCalibration(bool value) {
  enableCalibration_ = value;
}

bool ImageProcessor::isImageLoaded() {
  return vblocks_.image->isLoaded();
}
