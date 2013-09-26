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
#include <ctime>
#include "opencv2/core/core.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <boost/thread.hpp>
#include "LabicKinect.h"

namespace labic {
	
    class LabicCV {
    private:
		boost::thread m_Thread;
        Kinect *kinect;
		bool initialized;
        bool window_closed;
		int width;
		int height;
		uint16_t t_gamma[2048];
        bool previousSet, currentSet;
        bool stop;
        
		static const std::string input_window;
		static const std::string rgb_window;
		static const std::string rgb_t_window;
		static const std::string rgb_s_window;
		static const std::string depth_window;
		static const std::string rgbd_window;
        static const int DEPTH_RAW = 1;
        static const int DEPTH_MM = 2;
		static const int WINDOW_FLAGS = CV_WINDOW_AUTOSIZE;
        
        void keyboardHandler(int key);
        void generateDepthImage(uint16_t *depth, cv::Mat depthMat);
		cv::Vec3b depth_to_color(float raw_depth_value);
		
    public:
        cv::Mat rgbCurrent, rgbPrevious;
        uint16_t *depthCurrent, *depthPrevious;
        
        LabicCV(Kinect *_kinect, bool& _stop, int _width, int _height);
		void start();
        bool mainLoopPart(const int t);
		void join();
        void close();
        void init();
        void display();
        const bool isReady() const { return (previousSet && currentSet); }
        void restartState() { previousSet = currentSet = false; }
        static void showMatchesPreview(const cv::Mat& img1, const std::vector<cv::KeyPoint>& keypoints1, const cv::Mat& img2, const std::vector<cv::KeyPoint>& keypoints2, const std::vector<cv::DMatch>& matches1to2);

    };
}

#endif /* __LabicCV__ */
