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
using namespace Labic;

LabicCV::LabicCV(Kinect *_kinect, int _width, int _height) {
    kinect = _kinect;
	width = _width;
	height = _height;
    
    stop = false;
    initialized = false;
    
    for (unsigned int i=0; i<2048; i++) {
		float v = i/2048.0;
		v = powf(v, 3)* 6;
		t_gamma[i] = v*6*256;
	}
}

void LabicCV::init() {
    // Initialize windows
    
	namedWindow(depth_window, WINDOW_FLAGS);
//	resizeWindow(depth_window, width, height);
	moveWindow(depth_window, 50, 0);
	
	namedWindow(rgb_window, WINDOW_FLAGS);
//	resizeWindow(rgb_window, width, height);
	moveWindow(rgb_window, 50, 300);
    
    initialized = true;
}

void LabicCV::display() {
    Mat rgbMat(Size(width, height), CV_8UC3, Scalar(0));
	Mat depthMat(Size(width, height), CV_8UC3, Scalar(0));

    uint16_t *depth = (uint16_t*) malloc(sizeof(uint16_t)*width*height);
    int x, y;
    int depthValue;
    
	cout << "[LabicCV] Display started." << endl;
    if (!initialized) {
        cout << "[LabicCV] ERROR: did not call init(). Display finished." << endl;
        return;
    }
    
    do {
        kinect->getVideoMat(rgbMat);
        kinect->getDepth(depth);
        
        for (int i=0; i<width*height; i++) {
            y = i/width;
            x = i%width;
			
			depthValue = kinect->mmToRaw(depth[i]);
            
            depthMat.at<Vec3b>(y,x) = depth_to_color(depthValue);
//            cout << x << ", " << y << " = " << depthValue << "(" << depth[i] << " mm) -> " << depth_to_color(depthValue) << endl;

        }
        
        imshow(rgb_window, rgbMat);
        imshow(depth_window, depthMat);
        
    } while (!stop);
    
	cout << "[LabicCV] Display finished." << endl;
}

void LabicCV::displayRGB() {
    Mat rgbMat(Size(640,480), CV_8UC3, Scalar(0));
	
	cout << "[LabicCV] RGB window running." << endl;
	
	do {
        kinect->getVideoMat(rgbMat);
        
		imshow(rgb_window, rgbMat);
	} while (!stop);
	
	cout << "[LabicCV] RGB window finished." << endl;
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
    if (key == 27) {
        destroyAllWindows();
        stop = true;
    }
}

void LabicCV::start() {
	m_Thread = boost::thread(&LabicCV::display, this);
}

void LabicCV::join() {
    while (!stop) {
        keyboardHandler(waitKey(REFRESH_INTERVAL));
    }
    m_Thread.join();
}
