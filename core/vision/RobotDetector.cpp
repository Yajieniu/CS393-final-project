// pick white blobs and turn
#include <vision/RobotDetector.h>
#include <vision/BeaconDetector.h>
#include <memory/TextLogger.h>
#include <vision/Logging.h>

vector<Blob> RobotDetector::filterRoblobs(vector<Blob> &blobs, int size = 0) {
	vector<Blob> filtered;

	for (int i = 0; i < blobs.size(); ++i) {
		if((blobs[i].color != c_WHITE) || (blobs[i].color != c_ROBOT_WHITE))
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
		if ((rows_length[cutted_y] > width/2) ||  (rows_length[cutted_y] < width/16)) {
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

}

void RobotDetector::pickSURFArea(Blob &blob) {
	uint16_t height = (uint16_t) iparams_.height * 0.3;
    uint16_t width = (uint16_t) iparams_.width * 0.2;

    uint16_t x_start = max( (uint16_t) (blob.avgX - width), blob.xi);
    uint16_t x_end = min( (uint16_t) (x_start + width), blob.xf);
    uint16_t y_start = max( (uint16_t) (blob.avgY - height), blob.yf);
    uint16_t y_end = min( (uint16_t) (y_start + height), blob.yf);

}


void RobotDetector::findRobots(vector<Blob> &blobs) {
	if (camera_==Camera::BOTTOM) return;
	int width = iparams_.width;

	// pick white blobs
	vector<Blob> whiteBlobs = filterRoblobs(blobs, 100);

	// for each white blob
	for (int i = 0; i < whiteBlobs.size(); ++i) {

		filterWallAndLine(whiteBlobs[i]);
		
		// assume camera resolution 320 x 240
		// pick an area of at most (320 * 0.2) x (240 * 0.3)
		pickSURFArea(whiteBlobs[i]);

	}


}








