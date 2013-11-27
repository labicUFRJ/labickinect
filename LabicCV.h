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
#include "queue.h"
#include "KinectController.h"

namespace labic {
	class RGBDImage;
	
    class LabicCV {
    public:
        LabicCV(KinectController *_kinect, bool* _stop, Queue<RGBDImage>& q);
		void start() { m_Thread = boost::thread(&LabicCV::display, this); }
        void mainLoopPart(const int t);
		void join() { m_Thread.join(); }
        void close();
        void init();
        void display();
        void saveFrame();
        void setCaptureInterval(int interval) { captureInterval = interval; }
        void setCaptureHold(bool hold) { captureHold = hold; }
        const bool isReady() const { return currentSet; }
        void restartState() { currentSet = false; }
        void printStats() const;

    private:
        void keyboardHandler(int key);
        void generateDepthImage(const cv::Mat1f& depth, cv::Mat depthMat);
		cv::Vec3b depthToColor(float rawDepthValue);

		boost::thread m_Thread;
        KinectController *kinect;
        bool windowClosed;
        bool currentSet;
        bool* stop;
        int captureInterval;
        bool captureHold;
        bool startCapture;
		uint16_t t_gamma[2048];
        cv::Mat cameras;
        Queue<RGBDImage>& queue;

        long long unsigned int savedFrames;
        long long unsigned int processedFrames;
        double totalTime;
        
		static const std::string input_window;
    };
}

#endif /* __LABICKINECT_CV_H__ */
