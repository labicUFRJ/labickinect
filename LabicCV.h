//
//  LabicCV.h
//  LabicKinect
//
//  Created by Mario Cecchi on 8/22/13.
//
//

#ifndef __LabicCV__
#define __LabicCV__

#include <iostream>
#include <string>
#include "opencv2/core/core.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#ifndef BOOST_THREAD_INCLUDED
#define BOOST_THREAD_INCLUDED
#include <boost/thread.hpp>
#endif

#ifndef __LabicKinect__
#include "LabicKinect.h"
#endif

#endif /* defined(__LabicKinect__LabicCV__) */

namespace Labic {

    class LabicCV {
    private:
		boost::thread m_Thread;
        Kinect *kinect;
		bool initialized;
        bool stop;
        bool window_closed;
		int width;
		int height;
		cv::Mat rgb_image;
		cv::Mat depth_image;
		uint16_t t_gamma[2048];
		
		std::string input_window = "Kinect Input";
		std::string rgb_window = "RGB camera";
		std::string rgb_t_window = "Target RGB camera";
		std::string rgb_s_window = "Source RGB camera";
		std::string depth_window = "Depth camera";
		std::string rgbd_window = "RGBD Video";
        static const int REFRESH_INTERVAL = 1;
        static const int DEPTH_RAW = 1;
        static const int DEPTH_MM = 2;
		static const int WINDOW_FLAGS = CV_WINDOW_AUTOSIZE;
        
        void keyboardHandler(int key);
        void generateDepthImage(uint16_t *depth, cv::Mat depthMat);
		cv::Vec3b depth_to_color(float raw_depth_value);
		
    public:
        LabicCV(Kinect *_kinect, int _width, int _height);
		void start();
		void join();
        void init();
        void display();
    };
}