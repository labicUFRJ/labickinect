//
//  LabicCV.cpp
//  LabicKinect
//
//  Created by Mario Cecchi on 8/22/13.
//
//

#include "LabicCV.h"

using namespace std;
using namespace cv;
using namespace labic;

LabicCV::LabicCV(Kinect *_kinect, int _width, int _height) {
    kinect = _kinect;
	width = _width;
	height = _height;
    
    stop = false;
    window_closed = false;
    initialized = false;
    
    for (unsigned int i=0; i<2048; i++) {
		float v = i/2048.0;
		v = powf(v, 3)* 6;
		t_gamma[i] = v*6*256;
	}
    
    depthPrevious = (uint16_t*) malloc(sizeof(uint16_t)*width*height);
    depthCurrent = (uint16_t*) malloc(sizeof(uint16_t)*width*height);
    rgbPrevious = Mat(Size(width, height), CV_8UC3, Scalar(0));
    rgbCurrent = Mat(Size(width, height), CV_8UC3, Scalar(0));
    previousSet = currentSet = false;
}

void LabicCV::init() {
    namedWindow(input_window, WINDOW_FLAGS);
	
    initialized = true;
}

void LabicCV::display() {
    Mat rgbMat(Size(width, height), CV_8UC3, Scalar(0));
	Mat depthMat(Size(width, height), CV_8UC3, Scalar(0));
    Mat cameras(height, width*2, CV_8UC3);
    Mat left(cameras, Rect(0, 0, width, height));
    Mat right(cameras, Rect(width, 0, width, height));

    uint16_t *depth = NULL;
    
	cout << "[LabicCV] Display started" << endl;
    if (!initialized) {
        cout << "[LabicCV] ERROR: did not call init(). Display finished" << endl;
        return;
    }
    
    do {
		if (depth != NULL) free(depth);
        depth = (uint16_t*) malloc(sizeof(uint16_t)*width*height);
        
		/*
        kinect->getVideoMat(rgbMat);
        kinect->getDepth(depth);
		*/
		kinect->getFrame(rgbMat, depth);
        
        generateDepthImage(depth, depthMat);
        
		
        
        rgbMat.copyTo(left);
        depthMat.copyTo(right);
        
        putText(cameras, "W,S,X -> ADJUST TILT", Point(20,30), CV_FONT_HERSHEY_PLAIN, 0.8f, Scalar::all(0), 1, 8);
        
        if (!stop) imshow(input_window, cameras);
        
    } while (!stop);
    
    window_closed = true;

	cout << "[LabicCV] Display finished" << endl;
}

void LabicCV::generateDepthImage(uint16_t *depth, cv::Mat depthMat) {
    int x, y, i;
    int depthValue;
    
    clock_t t = clock();
    
    for (i=0; i<width*height; i++) {
        y = i/width;
        x = i%width;
        
        depthValue = kinect->mmToRaw(depth[i]);
        
        depthMat.at<Vec3b>(y,x) = depth_to_color(depthValue);
    }
    
    t = clock() - t;
    //cout << "[LabicCV] generateDepthImage time: " << 1000*((float)t)/CLOCKS_PER_SEC << " ms " << endl;
}

Vec3b LabicCV::depth_to_color(float raw_depth_value) {
    
    double r,g,b;
	int pval = t_gamma[(int)raw_depth_value];
    int lb = pval & 0xff;
    switch (pval>>8) {
        case 0:
            r = 255;
            g = 255-lb;
            b = 255-lb;
            break;
        case 1:
            r = 255;
            g = lb;
            b = 0;
            break;
        case 2:
            r = 255-lb;
            g = 255;
            b = 0;
            break;
        case 3:
            r = 0;
            g = 255;
            b = lb;
            break;
        case 4:
            r = 0;
            g = 255-lb;
            b = 255;
            break;
        case 5:
            r = 0;
            g = 0;
            b = 255-lb;
            break;
        default:
            r = 0;
            g = 0;
            b = 0;
            break;
    }
    
    return Vec3b(b,g,r);
}

void LabicCV::keyboardHandler(int key) {
    switch (key) {
        case 27:
            stop = true;
            destroyAllWindows();
            break;
        case '1':
            previousSet = kinect->getVideoMat(rgbPrevious);
            previousSet &= kinect->getDepth(depthPrevious);
            if (previousSet) cout << "[LabicCV] Previous state set" << endl;
            break;
        case '2':
            currentSet = kinect->getVideoMat(rgbCurrent);
            currentSet &= kinect->getDepth(depthCurrent);
            if (currentSet) cout << "[LabicCV] Current state set" << endl;
            break;
        case 'w':
            kinect->setTilt(+1.0);
            break;
        case 's':
            kinect->setTilt(0.0);
            break;
        case 'x':
            kinect->setTilt(-1.0);
            break;
        case ' ':
            
            break;
        case 63235: // right, bigger area

            break;
        case 63234: // left, smaller area

            break;
        case 63232: // up, area closer to top
            
            break;
        case 63233: // down, area closer to bottom
            
            break;

    }
}

void LabicCV::showMatchesPreview(const Mat& img1, const vector<KeyPoint>& keypoints1, const Mat& img2, const vector<KeyPoint>& keypoints2, const vector<DMatch>& matches1to2) {
    Mat outImg;
    namedWindow("RGB matched features");
    drawMatches(img1, keypoints1, img2, keypoints2, matches1to2, outImg);
    imshow("RGB matched features", outImg);
    //while (waitKey() != 27);
    //destroyWindow("RGB matched features");
}

void LabicCV::start() {
	m_Thread = boost::thread(&LabicCV::display, this);
}

bool LabicCV::mainLoopPart(const int t) {
    keyboardHandler(waitKey(t));
    return !(stop || window_closed);
}

void LabicCV::join() {
    m_Thread.join();
}

void LabicCV::close() {
    stop = true;
    destroyAllWindows();
    join();
}
