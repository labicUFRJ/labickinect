//
//  LabicCV.cpp
//  LabicKinect
//
//  Created by Mario Cecchi on 8/22/13.
//
//

#include <ctime>
#include "LabicCV.h"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;
using namespace labic;

const string LabicCV::input_window = "Kinect Input";
const string LabicCV::rgb_window = "RGB camera";
const string LabicCV::rgb_t_window = "Target RGB camera";
const string LabicCV::rgb_s_window = "Source RGB camera";
const string LabicCV::depth_window = "Depth camera";
const string LabicCV::rgbd_window = "RGBD Video";

LabicCV::LabicCV(KinectController *_kinect, bool* _stop) : kinect(_kinect), stop(_stop) {
    window_closed = false;
    initialized = false;
    
    for (unsigned int i=0; i<2048; i++) {
		float v = i/2048.0;
		v = pow(v, 3)* 6;
		t_gamma[i] = v*6*256;
	}
    
    cameras = Mat(height, width*2, CV_8UC3);
    previousSet = currentSet = false;

    namedWindow(input_window);
    initialized = true;
    framesSaved = 0;
}

void LabicCV::init() {
}

void LabicCV::display() {
    Mat rgbMat(Size(width, height), CV_8UC3, Scalar(0));
	Mat depthMat(Size(width, height), CV_8UC3, Scalar(0));
    Mat left(cameras, Rect(0, 0, width, height));
    Mat right(cameras, Rect(width, 0, width, height));

    uint16_t *depth = NULL;
    
	cout << "[LabicCV] Display started" << endl;
    if (!initialized) {
        cout << "[LabicCV] ERROR: did not call init(). Display finished" << endl;
        return;
    }

    do {
        kinect->getRGBDImage(rgbdDisplay);
        
        generateDepthImage(rgbdDisplay.depth(), depthMat);
        
        rgbdDisplay.rgb().copyTo(left);
        depthMat.copyTo(right);
        
        //putText(cameras, "W,S,X -> ADJUST TILT", Point(20,30), CV_FONT_HERSHEY_PLAIN, 0.8f, Scalar::all(0), 1, 8);
        
        //cout << "Loop timestamp " << rgbdDisplay.timestamp << endl;
    } while (!*stop);
    
    window_closed = true;

	cout << "[LabicCV] Display finished" << endl;
}

void LabicCV::generateDepthImage(const Mat1f& depth, Mat depthMat) {
    int x, y, i;
    int depthValue;

    clock_t t = clock();

    for (y=0; y<height; y++) {
    	for (x=0; x<width; x++) {
    		depthValue = mmToRaw(depth(y,x));
    		depthMat.at<Vec3b>(y,x) = depth_to_color(depthValue);
    	}

    }

    t = clock() - t;
    //cout << "[LabicCV] generateDepthImage time: " << 1000*((float)t)/CLOCKS_PER_SEC << " ms " << endl;
}

void LabicCV::generateDepthImage(uint16_t *depth, Mat depthMat) {
    int x, y, i;
    int depthValue;
    
    clock_t t = clock();
    
    for (i=0; i<width*height; i++) {
        y = i/width;
        x = i%width;
        
        depthValue = mmToRaw(depth[i]);
        
        depthMat.at<Vec3b>(y,x) = depth_to_color(depthValue);
    }
    
    t = clock() - t;
    //cout << "[LabicCV] generateDepthImage time: " << 1000*((float)t)/CLOCKS_PER_SEC << " ms " << endl;
}

Vec3b LabicCV::depth_to_color(float rawDepthValue) {
    double r,g,b;
	int pval = t_gamma[(int)rawDepthValue];
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
            *stop = true;
            destroyAllWindows();
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
			while (!currentSet) {
				//currentSet = kinect->getFrame(rgbCurrent, depthCurrent);
				currentSet = kinect->getRGBDImage(rgbdCurrent);
			}
			framesSaved++;
			cout << "[LabicCV] " << framesSaved << " frames saved" << endl;
            break;
        default:
        	break;

    }
}

void LabicCV::start() {
	m_Thread = boost::thread(&LabicCV::display, this);
}

void LabicCV::mainLoopPart(const int t) {
	if (*stop) return ;
	imshow(input_window, cameras);
    keyboardHandler(waitKey(t));
}

void LabicCV::join() {
    m_Thread.join();
}

void LabicCV::close() {
    *stop = true;
    destroyAllWindows();
    join();
}
