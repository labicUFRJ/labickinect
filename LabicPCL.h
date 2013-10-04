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

namespace labic {
	
	class LabicPCL {
		
	private:
		boost::thread m_Thread;
		KinectController *kinect;
        bool* stop;
        void generateDepthCloud(uint16_t *depth);
		
	public:
        pcl::visualization::PCLVisualizer viewer;
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::PointCloud<pcl::PointXYZRGB> liveCloud;
        int viewPort;
        
        LabicPCL(KinectController *_kinect, bool* _stop);
		void start();
        bool mainLoopPart(const int t);
		void join();
        void close();
        
        void addCameras(const std::vector<cv::Mat>&         T,
                        const std::vector<cv::Mat>&         R);
        
        void updateCloud(std::vector< cv::Point3d >& objPoints, cv::Mat img);
        
        void display();
        
    };
}

#endif /* __LABICKINECT_PCL_H__ */
