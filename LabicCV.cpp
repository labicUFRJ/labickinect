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
	
	namedWindow(depth_window, WINDOW_FLAGS);
	resizeWindow(depth_window, width, height);
	moveWindow(depth_window, 50, 0);
	
	namedWindow(rgb_window, WINDOW_FLAGS);
	resizeWindow(rgb_window, width, height);
	moveWindow(rgb_window, 50, 300);

}

void LabicCV::display() {
	cout << "[LabicCV] Display started." << endl;
	display_rgb_window();
	cout << "[LabicCV] Display finished." << endl;
}

void LabicCV::display_rgb_window() {
	vector<uint8_t> rgb_buffer;
	int key;
	
	cout << "[LabicCV] RGB window starting..." << endl;
	
	do {
		if (!kinect->getRGB(rgb_buffer)) continue;
		
		rgb_image = Mat(height, width, CV_8UC3, &rgb_buffer);
//		cvtColor(rgb_image, rgb_image, CV_RGB2BGR);
		imshow(rgb_window, rgb_image);
		
		key = waitKey(REFRESH_INTERVAL);
        if (key == 27) {
            //kinect_stop = 1;
            break;
        }
	} while (1);
	
	cout << "[LabicCV] RGB window finished." << endl;
}

Vec3b LabicCV::depth_to_color_mm(float depth_value) {
    // convert from mm to raw depth to calculate corresponding color
    //depth_value = kinect->mm_to_raw(depth_value);
    
    double r,g,b;
	int pval = t_gamma[(int)depth_value];
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


void LabicCV::start() {
	m_Thread = boost::thread(&LabicCV::display, this);
}

void LabicCV::join() {
    m_Thread.join();
}
