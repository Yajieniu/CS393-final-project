#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <kinematics/ForwardKinematics.h>
#include <common/RobotDimensions.h>
#include <common/Profiling.h>
#include <memory/TextLogger.h>
#include <vision/CameraMatrix.h>
#include <vision/VisionBlocks.h>
#include <common/RobotInfo.h>
#include <common/RobotCalibration.h>
#include <vision/structures/BallCandidate.h>
#include <math/Pose3D.h>
#include <vision/structures/VisionParams.h>

class BallDetector;
class Classifier;
class BeaconDetector;

struct block_t {
    block_t* parent;
    short length;
    short x;
    short y;
    short minX;
    short minY;
    short maxX;
    short maxY;
    double meanX;
    double meanY;
    int count;
    Color color;
};

/// @ingroup vision
class ImageProcessor {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
    ImageProcessor(VisionBlocks& vblocks, const ImageParams& iparams, Camera::Type camera);  // how to use vblocks
    ~ImageProcessor();  // what is this
    void processFrame(); 
    void init(TextLogger*);
    void SetColorTable(unsigned char*);
    std::unique_ptr<BeaconDetector> beacon_detector_;
    std::unique_ptr<Classifier> color_segmenter_;
    unsigned char* getImg();
    unsigned char* getSegImg();
    unsigned char* getColorTable();
    bool isRawImageLoaded();
    int getImageHeight();
    int getImageWidth();
    const ImageParams& getImageParams() const { return iparams_; }
    const CameraMatrix& getCameraMatrix();
    void setCalibration(const RobotCalibration& calibration);
    void enableCalibration(bool value);
    void updateTransform();
    std::vector<BallCandidate*> getBallCandidates();
    BallCandidate* getBestBallCandidate();
    bool isImageLoaded();
    void detectBlob();
    bool findBall(int& imageX, int& imageY);
    WorldObject* getBall();
  private:
    int getTeamColor();
    double getCurrentTime();

    VisionBlocks& vblocks_;
    const ImageParams& iparams_;
    Camera::Type camera_;
    CameraMatrix cmatrix_;
    
    VisionParams vparams_;
    unsigned char* color_table_;
    TextLogger* textlogger;

    float getHeadPan() const;
    float getHeadTilt() const;
    float getHeadChange() const;
    
    std::unique_ptr<RobotCalibration> calibration_;
    bool enableCalibration_;

    //void saveImg(std::string filepath);
    int topFrameCounter_ = 0;
    int bottomFrameCounter_ = 0;

    void markBall(int, int, int);
    void markGoal(int, int);
    void markBeacon(WorldObjectType, int, int, int);

    void RLE(block_t* blocks);
    void mergeRow(block_t*, block_t*);
    void mergeBlock(block_t*, block_t*);
    void initBlock(block_t*, int, int, int, unsigned char);
    block_t* findBlockParent(block_t*);
    void unionBlock(block_t*, block_t*);

    // False positive filter
    bool generalBlobFilter(block_t*);
    bool lookLikeBall(block_t*);
    bool lookLikeGoal(block_t*);
    bool lookLikeBeacon(block_t*, WorldObjectType, int&, int&, int&);

};

#endif
