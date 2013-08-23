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
#include <boost/thread.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#ifndef __LabicKinect__
#include "LabicKinect.h"
#endif

#endif /* defined(__LabicKinect__LabicCV__) */

namespace Labic {

    class LabicCV {
    private:
		boost::thread m_Thread;
        Kinect *kinect;
		
		int width;
		int height;
		cv::Mat rgb_image;
		cv::Mat depth_image;
		uint16_t t_gamma[2048];
		
		std::string rgb_window = "RGB Camera";
		std::string rgb_t_window = "Target RGB Camera";
		std::string rgb_s_window = "Source RGB Camera";
		std::string depth_window = "Depth";
		std::string rgbd_window = "RGBD Video";
		static const int WINDOW_FLAGS = CV_WINDOW_NORMAL | CV_GUI_EXPANDED | CV_WINDOW_KEEPRATIO;
		static const int REFRESH_INTERVAL = 1;

		void display_rgb_window();
		void display_depth_window();
		cv::Vec3b depth_to_color_mm(float depth_value);
		
    public:
        LabicCV(Kinect *_kinect, int _width, int _height);
		void start();
		void join();
        void display();
    };
}