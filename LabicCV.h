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
#include "queue.h"

namespace labic {
	
    class LabicCV {
    public:
        LabicCV(KinectController *_kinect, bool* _stop, FrameQueue& q);
		void start();
        void mainLoopPart(const int t);
		void join();
        void close();
        void init();
        void display();
        void saveFrame();
        void setCaptureInterval(int interval) { captureInterval = interval; }
        const bool isReady() const { return currentSet; }
        void restartState() { currentSet = false; }

    private:
        void keyboardHandler(int key);
        void generateDepthImage(const cv::Mat1f& depth, cv::Mat depthMat);
		cv::Vec3b depthToColor(float rawDepthValue);

		boost::thread m_Thread;
        KinectController *kinect;
		bool initialized;
        bool windowClosed;
        bool currentSet;
        bool* stop;
        int captureInterval;
		uint16_t t_gamma[2048];
        cv::Mat cameras;
        FrameQueue& queue;
        
		static const std::string input_window;
        static const int DEPTH_RAW = 1;
        static const int DEPTH_MM = 2;
		static const int WINDOW_FLAGS = CV_WINDOW_AUTOSIZE;
        
    };
}

#endif /* __LABICKINECT_CV_H__ */
