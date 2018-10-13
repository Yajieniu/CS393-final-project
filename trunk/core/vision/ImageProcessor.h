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
#include <unordered_map>
#include <vector>

class BallDetector;
class Classifier;
class BeaconDetector;

struct RLE {
    int lcol;
    int rcol;
    int parent;
    int curr;
    int npixels;
    int color;
    int rank;
	int xi;
	int xf;
	int yi;
	int yf;
	int xsum;
	int ysum;

    RLE(int y, int l, int r, int idx, int c, int ystep) {
        lcol = l;
        rcol = r;
        parent = idx;
        curr = idx;
        npixels = (r - l + 1) * ystep;
        color = c;
        rank = 1;
		xi = l; xf = r;
		yi = y; yf = y + ystep - 1;
		xsum = ((r + l) / 2) * (r - l + 1) * ystep;
		ysum = y * (r - l + 1) * ystep;
    }
};

struct RLECompare {
	bool operator()(RLE* a, RLE* b) {
		return a->npixels > b->npixels;
	}
};

Blob makeBlob(RLE* r);

/// @ingroup vision
class ImageProcessor {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
    ImageProcessor(VisionBlocks& vblocks, const ImageParams& iparams, Camera::Type camera);
    ~ImageProcessor();
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
    int getParent(int idx); 
    void mergeBlobs(int idx1, int idx2); 
	vector<RLE*> getRLERow(int y, int width, int &start_idx); 
	void mergeEncodings(vector<RLE*> &prev_encoding, vector<RLE*> &encoding); 
	void calculateBlobs();
    void detectBall();
    void findBall(int& imageX, int& imageY);
    void detectGoal();
    void findGoal(int& imageX, int& imageY);
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
    
    // blob store
    unordered_map<int, RLE*> rle_ptr;
    vector<Blob> detected_blobs;

    // Ball detection
    vector<BallCandidate*> ball_candidates;
};

#endif
