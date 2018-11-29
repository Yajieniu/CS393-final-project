#include <vision/BeaconDetector.h>
#include <memory/TextLogger.h>
#include <vision/Logging.h>

using namespace Eigen;

BeaconDetector::BeaconDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

unsigned char* BeaconDetector::getSegImg(){
    if(camera_ == Camera::TOP)
        return vblocks_.robot_vision->getSegImgTop();
    return vblocks_.robot_vision->getSegImgBottom();
}

vector<Blob> filterByAspectRatio(vector<Blob> blobs) {
    vector<Blob> ret;
    for(int i = 0; i < blobs.size(); ++i) {
        double aspect_ratio = calculateBlobAspectRatio(blobs[i]);

        if (aspect_ratio < ASPECT_RATIO_LOW_BOUND || aspect_ratio > ASPECT_RATIO_HIGH_BOUND)
            continue;

        ret.push_back(blobs[i]);
    }
    return ret;
}

vector<Blob> filterByDensity(vector<Blob> blobs) {
    vector<Blob> ret;
    for(int i = 0; i < blobs.size(); ++i) {
        double area = calculateBlobArea(blobs[i]);
        double density = blobs[i].lpCount / area;
        if(density < DENSITY_LOW_BOUND)
            continue;
        ret.push_back(blobs[i]);
    }
    return ret;
}

vector<pair<Blob, Blob> > makeBeaconPairs(vector<Blob> &tblobs, vector<Blob> &bblobs) {
    vector<pair<Blob, Blob> > beacons;
    for(int i = 0; i < tblobs.size(); ++i) {
        for(int j = 0; j < bblobs.size(); ++j) {
            if(tblobs[i].avgY > bblobs[j].avgY)
                continue;
            if(tblobs[i].avgX > bblobs[j].xf || tblobs[i].avgX < bblobs[j].xi)
                continue;
            if(bblobs[j].avgX > tblobs[i].xf || bblobs[j].avgX < tblobs[i].xi)
                continue;
            if(bblobs[j].yi - tblobs[i].yf > VERTICAL_SEPARATION_HIGH_BOUND)
                continue;

        #ifdef ENABLE_AREA_SIM_FILTERING
            double tarea = calculateBlobArea(tblobs[i]);
            double barea = calculateBlobArea(bblobs[j]);
            double areaSim = tarea / barea;
            if(areaSim > AREA_SIM_HIGH_BOUND || areaSim < AREA_SIM_LOW_BOUND)
                continue;
            // cout << "AS: " << areaSim << endl;
        #endif

            tblobs[i].invalid = false;
            bblobs[j].invalid = false;
            beacons.push_back(make_pair(tblobs[i], bblobs[j]));
        }
    }
    return beacons;
}

bool BeaconDetector::validateInverted(pair<Blob, Blob> &bblob) {
    unsigned char* segImg = getSegImg();
    const int width = iparams_.width;
    const int height = iparams_.height;
    const int xstep = (1 << iparams_.defaultHorizontalStepScale);
    const int ystep = (1 << iparams_.defaultVerticalStepScale);
    
    int startX = (bblob.first.xi + bblob.second.xi) / 2;
    startX -= (startX % xstep);
    int startY = bblob.second.yf;
    startY -= (startY % ystep);
    int dx = (bblob.first.dx + bblob.second.dx) / 2;
    int dy = (bblob.first.dy + bblob.second.dy) / 2;
    int endX = min(width - 1, startX + dx);
    int endY = min(height - 1, startY + dy);

    int tot_count = 1;
    int white_count = 0;
    for(int i = startX; i <= endX; i += xstep) {
        for(int j = startY; j <= endY; j += ystep) {
            auto c = static_cast<Color>(segImg[j * width + i]);
            white_count += c == c_WHITE ? 1 : 0;
            tot_count++;
        }
    }
    double ratio = (double) white_count / tot_count;
    // cout << white_count << " " << tot_count << endl;
    // cout << "White Ratio: " << ratio << endl;
    return ratio >= WHITE_BELOW_BEACON_LOW_BOUND;
}

bool BeaconDetector::validateUp(pair<Blob, Blob> &bblob) {
    unsigned char* segImg = getSegImg();
    const int width = iparams_.width;
    const int height = iparams_.height;
    const int xstep = (1 << iparams_.defaultHorizontalStepScale);
    const int ystep = (1 << iparams_.defaultVerticalStepScale);

    int startX = (bblob.first.xi + bblob.second.xi) / 2;
    startX -= (startX % xstep);
    int dx = (bblob.first.dx + bblob.second.dx) / 2;
    int dy = (bblob.first.dy + bblob.second.dy) / 2;
    int startY = max(0, bblob.first.yi - dy);
    startY -= (startY % ystep);
    int endX = min(width - 1, startX + dx);
    int endY = min(height - 1, startY + dy);

    if(bblob.first.yi - startY < dy * 0.4)
        return true;

    int tot_count = 1;
    map<Color, int> colorCount = {
        {c_BLUE, 0},
        {c_YELLOW, 0},
        {c_PINK, 0}
    };

    for(int i = startX; i <= endX; i += xstep) {
        for(int j = startY; j <= endY; j += ystep) {
            auto c = static_cast<Color>(segImg[j * width + i]);
            tot_count++;
            if(colorCount.find(c) != colorCount.end()) {
                colorCount[c]++;
            }
        }
    }

    for(auto color: colorCount) {
        double ratio = (double) color.second / tot_count;
        // cout << COLOR_NAME(color.first) << " " << ratio << endl;
        if(ratio >= COLOR_ABOVE_BEACON_HIGH_BOUND)
            return false;
    }

    return true;
}

pair<Blob, Blob> BeaconDetector::findBeaconsOfType(const vector<Blob> &tb, const vector<Blob> &bb) {

    // check if the aspect ratio is within ASPECT_RATIO_LOW_BOUND & ASPECT_RATIO_HIGH_BOUND
    auto tblobs = filterByAspectRatio(tb);
    auto bblobs = filterByAspectRatio(bb);
    // check if the density is greater than DENSITY_LOW_BOUND
    tblobs = filterByDensity(tblobs);
    bblobs = filterByDensity(bblobs);

    auto beacons = makeBeaconPairs(tblobs, bblobs);

    for(int i = 0; i < beacons.size(); ++i) {
        if(!validateInverted(beacons[i]) || !validateUp(beacons[i]))
            continue;
        return beacons[i];
    }

    // no valid beacon found return invalid
    Blob b;
    b.invalid = true;
    return make_pair(b, b);
}

double density(Blob &b) {
    double area = calculateBlobArea(b);
    double density = b.lpCount / area;

    return density;
}

void BeaconDetector::findBeacons(vector<Blob> &blobs) {
    if(camera_ == Camera::BOTTOM) return;
    static map<WorldObjectType,int> heights = {
        { WO_BEACON_BLUE_YELLOW,    300 },
        { WO_BEACON_YELLOW_BLUE,    300 },
        { WO_BEACON_BLUE_PINK,      200 },
        { WO_BEACON_PINK_BLUE,      200 },
        { WO_BEACON_PINK_YELLOW,    200 },
        { WO_BEACON_YELLOW_PINK,    200 }
    };
    static map<WorldObjectType, vector<Color> > beacons = {
        { WO_BEACON_BLUE_YELLOW,    { c_BLUE, c_YELLOW } },
        { WO_BEACON_YELLOW_BLUE,    { c_YELLOW, c_BLUE } },
        { WO_BEACON_BLUE_PINK,      { c_BLUE, c_PINK } },
        { WO_BEACON_PINK_BLUE,      { c_PINK, c_BLUE } },
        { WO_BEACON_PINK_YELLOW,    { c_PINK, c_YELLOW } },
        { WO_BEACON_YELLOW_PINK,    { c_YELLOW, c_PINK } }
    };

    map<Color, vector<Blob> > colorBlobs = {
        { c_BLUE,       filterBlobs(blobs, c_BLUE, 50)     },
        { c_YELLOW,     filterBlobs(blobs, c_YELLOW, 50)   },
        { c_PINK,       filterBlobs(blobs, c_PINK, 50)     },
        { c_WHITE,      filterBlobs(blobs, c_WHITE, 50)    }
    };

    for(auto beacon : beacons) {
        auto& object = vblocks_.world_object->objects_[beacon.first];
        auto ctop = beacon.second[0];
        auto cbottom = beacon.second[1];

        pair<Blob, Blob> bblob = findBeaconsOfType(colorBlobs[ctop], colorBlobs[cbottom]);

        if(bblob.first.invalid || bblob.second.invalid) {
            object.seen = false;
            continue;
        }

        double aspect_ratio = (calculateBlobAspectRatio(bblob.first) + calculateBlobAspectRatio(bblob.second)) / 2.0;

        object.imageCenterX = (bblob.first.avgX + bblob.second.avgX) / 2;
        object.imageCenterY = (bblob.first.avgY + bblob.second.avgY) / 2;

        double bwidth = (bblob.first.dx + bblob.second.dx) / 2.0;
        double bheight = (bblob.first.dy + bblob.second.dy) / 2.0;
        double bdistance = (cmatrix_.getWorldDistanceByWidth(bwidth, 110.0) + cmatrix_.getWorldDistanceByHeight(bheight, 100.0)) / 2.0;

        auto position = cmatrix_.getWorldPosition(object.imageCenterX, object.imageCenterY, heights[beacon.first]);
        object.visionDistance = bdistance;
        object.visionBearing = cmatrix_.bearing(position);
        object.seen = true;
        object.fromTopCamera = (camera_ == Camera::TOP);

        if(aspect_ratio <= OCCLUDED_ASPECT_RATIO_HIGH_BOUND) {
            object.occluded = true;
        }
        else {
            object.occluded = false;
        }
        
        // cout << "Total AR: " << aspect_ratio << endl;
        // cout << "AR: " << calculateBlobAspectRatio(bblob.first) << ", " << calculateBlobAspectRatio(bblob.second) << endl;
        // cout << "density: " << density(bblob.first) << ", " << density(bblob.second) << endl;"
        // cout << "saw " << getName(beacon.first) << " at (" << object.imageCenterX << "," << object.imageCenterY << ") with calculated distance " << object.visionDistance << endl;
    }
    // cout << endl << endl;
}
