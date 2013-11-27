//
//  pcl.h
//  LabicKinect
//
//  Created by Mario Cecchi on 05/07/13.
//  Copyright (c) 2013 Mario Cecchi. All rights reserved.
//

#ifndef __LABICKINECT_PCL_H__
#define __LABICKINECT_PCL_H__

#include <pcl/visualization/pcl_visualizer.h>
#include "common.h"
#include "KinectController.h"
#include "queue.h"
#include "RGBDImage.h"

namespace labic {
	
	class LabicPCL {
	public:
        LabicPCL(KinectController *_kinect, bool* _stop, Queue<RGBDImage>& q);
		void start();
		void join();
        void close();
        
        void addCameras(const std::vector<cv::Mat>& T,
                        const std::vector<cv::Mat>& R);
        
        void updateCloud(std::vector< cv::Point3d >& objPoints, cv::Mat img);
        
        void display();

	private:
		boost::thread m_Thread;
		KinectController *kinect;
        bool* stop;
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        Queue<RGBDImage>& queue;
        int viewPort;
        void generateDepthCloud(uint16_t *depth);

    };
}

#endif /* __LABICKINECT_PCL_H__ */
