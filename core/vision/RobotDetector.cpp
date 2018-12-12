// pick white blobs and turn
#include <vision/RobotDetector.h>
#include <vision/BeaconDetector.h>
#include <vision/ImageProcessor.h>
#include <memory/TextLogger.h>
#include <common/ColorConversion.h>
#include <memory/ImageBlock.h>
#include <vision/Logging.h>
#include <stdio.h>
#include <iostream>


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>



using namespace cv;
// using namespace features2d;

RobotDetector::RobotDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

vector<Blob> RobotDetector::filterRoblobs(vector<Blob> &blobs, int size = 0) {
	vector<Blob> filtered;

	for (int i = 0; i < blobs.size(); ++i) {
		if((blobs[i].color != c_WHITE) && (blobs[i].color != c_ROBOT_WHITE))
            continue;
        if(blobs[i].lpCount < size)
            continue;
        filtered.push_back(blobs[i]);
    }
    sort(filtered.begin(), filtered.end(), BlobCompare);
    return filtered;
}

unsigned char* RobotDetector::getSegImg(){
  if(camera_ == Camera::TOP)
    return vblocks_.robot_vision->getSegImgTop();
  return vblocks_.robot_vision->getSegImgBottom();
}

void RobotDetector::filterWallAndLine(Blob &whiteBlob) {

	uint16_t xi, xf, dx, yi, yf, dy;
    uint16_t width = (uint16_t) iparams_.width;
    uint16_t height = (uint16_t) iparams_.height;

	
	xi = whiteBlob.xi;
	xf = whiteBlob.xf;
	dx = whiteBlob.dx;
	yi = whiteBlob.yi;
	yf = whiteBlob.yf;
	dy = whiteBlob.dy;

	unsigned char* originalImg = getSegImg();
	unsigned char cuttedImg[whiteBlob.dx * whiteBlob.dy] = {};

	// assuming y is the same for each row
	unsigned char rows_length[dy] = { };
	unsigned char col_height[dx] = { }; 

	double white_pixel_sum = 0;

	// find all white pixels in the cutted Image
	// and calculate horizontal and vertical white pixel number  
	for (int y = yi; y <= yf; ++y) {
		int cutted_y = y - yi;

		for (int x = xi; x <= xf; ++x) {
			auto color = originalImg[y * width + x];
			if((color == c_WHITE) || (color == c_ROBOT_WHITE)) {
				cuttedImg[cutted_y * dx + (x - xi)] = 1;
				rows_length[cutted_y] += 1;
			}
		}

		// filter out wall and vertical lines
		if ((rows_length[cutted_y] > width/3) ||  (rows_length[cutted_y] < 30)) {
			rows_length[cutted_y] = 0;
		}

		for (int x = xi; x <= xf; ++x) {
			if (cuttedImg[cutted_y * dx + (x - xi)] == 1) {
				if (rows_length[cutted_y]) {
					col_height[cutted_y] += 1;
					white_pixel_sum += 1;
				}
				else {
					cuttedImg[cutted_y * dx + (x - xi)] = 0;
				}	
			}
		}
		
	}
	
	// refine the range of white blob
	for (int y = yi; y <= yf; ++y) {
		if (rows_length[y-yi]) {
			yi = y;
			break;
		}
	}

	for (int y = yf; y >= yi; --y) {
		if (rows_length[y-yi]) {
			yf = y;
			break;
		}
	}

	for (int x = xi; x <= xf; ++x) {
		if (col_height[x-xi]) {
			xi = x;
			break;
		}
	}

	for (int x = xf; x >= xi; --x) {
		if (col_height[x-xi]) {
			xf = x;
			break;
		}
	}

	dx = xf - xi;
	dy = yf - yi;

	double sum1 = 0; 
	double sum2 = 0;
	uint16_t avgY;
	uint16_t avgX;

	// update average Y
	for (int y = yi; y <= yf; ++y) {
		sum1 += rows_length[y-yi];
		if (sum1 >= white_pixel_sum/2) {
			avgY = y;
			break;
		}
	}

	// update average X
	for (int x = xi; x <= xf; ++x) {
		sum2 += col_height[x-xi];
		if (sum2 >= white_pixel_sum/2) {
			avgX = x;
			break;
		}
	}	

	whiteBlob.xi = xi;
	whiteBlob.xf = xf;
	whiteBlob.dx = xf - xi + 1;
	whiteBlob.yi = yi;
	whiteBlob.yf = yf;
	whiteBlob.dy = yf - yi + 1;

	whiteBlob.avgX = avgX;
	whiteBlob.avgY = avgY;
	// finish refining the range of white_blob[i]


	//update center Y
	// auto centerY = std::distance(std::begin(col_height), std::max_element(std::begin(col_height), std::end(col_height)));
	whiteBlob.avgX = updateCenterY(col_height)+xi;


}

uint16_t RobotDetector::updateCenterY(unsigned char* col_height) {
	uint16_t highest = 0;
	int length = (sizeof(col_height)/sizeof(*col_height));

	for (int i = 0; i< length; i++) {
		if (highest < col_height[i]) {
			highest = col_height[i];
		}
	}

	uint16_t index = 0;
	for (int i = 0; i< length; i++) {
		if (highest == col_height[i]) {
			index = i;
		}
	}

	return index;
}

float RobotDetector::SURFTest(Blob &blob, unsigned char* rawImage) {
	
    //-- Step 1: Detect the keypoints using SURF Detector
  	double minHessian = 400;

  	cv::SurfFeatureDetector detector(minHessian);

  	std::vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;

	string path = "~/home/yajieniu/nao/trunk/core/image1.png";
    cv::Mat cvImageBank = cv::imread(path, IMREAD_GRAYSCALE);
    // may need to turn gray
    cv::Mat cvImageCurrent = color::rawToMat((const) rawImage, iparams);
    cv::Mat cvImageCurrentGray = cv::cvtcolor(cvImageCurrent, cvImageCurrentGray,
    		CV_BGR2GRAY);

	cv::Mat img1 = cvImageBank;
	cv::Mat img2 = cvImageCurrentGray;
	    
    detector->detectAndCompute( img1, noArray(), keypoints1, descriptors1 );
    detector->detectAndCompute( img2, noArray(), keypoints2, descriptors2 );

	  //-- Step 2: Matching descriptor vectors using FLANN matcher
	FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    matcher.match( descriptors_1, descriptors_2, matches );
    double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_1.rows; i++ )
    { 
    	double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    //compute good_matches
    std::vector< DMatch > good_matches;
    for( int i = 0; i < descriptors_1.rows; i++ )
    { if( matches[i].distance <= max(2*min_dist, 0.02) )
      { good_matches.push_back( matches[i]); }
    }



}


std::vector<RobotCandidate*> RobotDetector::findRobots(vector<Blob> &blobs, 
	std::vector<RobotCandidate*> robot_candidates, unsigned char* rawImage,
	 const ImageParams& iparams_) {

	iparams = iparams_;

	uint16_t x_threshold = 100;//(uint16_t) (iparams_.height /10 * 3);
    uint16_t y_threshold = 100;//(uint16_t) (iparams_.width / 10 * 2);

	// int width = iparams_.width;
	float confidence = 0;
	float threshold = 0.6;
	robot_candidates.clear();
	Blob blob;

	// pick white blobs
	vector<Blob> whiteBlobs = filterRoblobs(blobs, 900);
	// vector<Blob> whiteBlobs = filterBlobs(blobs, c_ROBOT_WHITE, 100);

	// for each white blob
	for (int i = 0; i < whiteBlobs.size(); ++i) {
	// for (int i = 0; i < blobs.size(); ++i) {
		std::cout << i << endl;

		filterWallAndLine(whiteBlobs[i]);


		
		// assume camera resolution 320 x 240
		// pick an area of at most 64 x 48
		confidence = 0.7; //SURFTest(whiteBlobs[i]);

		if (confidence > threshold) {
			RobotCandidate* robot = new RobotCandidate();

		    
			robot->xi = whiteBlobs[i].xi;
			robot->xf = whiteBlobs[i].xf;
			robot->yi = whiteBlobs[i].yi;
			robot->yf = whiteBlobs[i].yf;

			robot->avgX = whiteBlobs[i].avgX;
			robot->avgY = whiteBlobs[i].avgY;


			// robot->xmin = (uint16_t)(iparams_.width/4);
			// robot->xmax = (uint16_t)(3*iparams_.width/4);
			// robot->ymin = (uint16_t)(iparams_.height/4);
			// robot->ymax = (uint16_t)(3*iparams_.height/4);

			// if (whiteBlobs[i].dy > y_threshold) {
			// 	robot->yi = max((uint16_t)(robot->avgY-y_threshold/2), (uint16_t) 0);
			// 	robot->yf = min((uint16_t)(robot->avgY+y_threshold/2), (uint16_t) (iparams_.height));
			// }
			
			// if (whiteBlobs[i].dx > x_threshold) {
			// 	robot->xi = max((uint16_t)(robot->avgX-x_threshold/2), (uint16_t) 0);
			// 	robot->xf = min((uint16_t)(robot->avgX+x_threshold/2), (uint16_t) (iparams_.width));
			// }
			
		    // robot->xmin = max( (uint16_t) (robot->avgX - width/2), robot->xi);
		    // robot->xmax = min( (uint16_t) (robot->avgX + width/2), robot->xf);
		    // robot->ymin = max( (uint16_t) (robot->avgY - height/2), robot->yi);
		    // robot->ymax = min( (uint16_t) (robot->avgY + height/2), robot->yf);

		    // add the confidence value for robot candidates
			robot->confidence = confidence;

			// approximate position
		    Position p = cmatrix_.getWorldPosition(robot->avgX, robot->yf);
		    robot->groundDistance = cmatrix_.groundDistance(p);

		    // [TODO] add the blob on the heap and send
		    robot->blob = NULL;

		    // [TODO] add the absolute and relative positions
		    robot_candidates.push_back(robot);
			
		}

	}
	return robot_candidates;
}








