//
//  LabicCV.h
//  LabicKinect
//
//  Created by Mario Cecchi on 8/22/13.
//
//

#ifndef __LABICKINECT_CV_H__
#define __LABICKINECT_CV_H__

#include <string>
#include "opencv2/highgui/highgui.hpp"
#include "common.h"
#include "KinectController.h"
#include "RGBDImage.h"

namespace labic {
	
    class LabicCV {
    private:
		boost::thread m_Thread;
        KinectController *kinect;
		bool initialized;
        bool window_closed;
		int framesSaved;
		uint16_t t_gamma[2048];
        bool previousSet, currentSet;
        bool* stop;
        cv::Mat cameras;
        
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
        void generateDepthImage(const cv::Mat1f& depth, cv::Mat depthMat);
        void generateDepthImage(uint16_t *depth, cv::Mat depthMat);
		cv::Vec3b depth_to_color(float rawDepthValue);
		
    public:
        RGBDImage rgbdDisplay, rgbdCurrent;
        
        LabicCV(KinectController *_kinect, bool* _stop);
		void start();
        void mainLoopPart(const int t);
		void join();
        void close();
        void init();
        void display();
        const bool isReady() const { return currentSet; }
        void restartState() { currentSet = false; }

    };
}

#endif /* __LABICKINECT_CV_H__ */
