//
//  pcl.h
//  LabicKinect
//
//  Created by Mario Cecchi on 05/07/13.
//  Copyright (c) 2013 Mario Cecchi. All rights reserved.
//

#ifndef __LabicPCL__
#define __LabicPCL__

#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include "opencv2/core/core.hpp"
#include <boost/thread.hpp>
#include "LabicKinect.h"

namespace labic {
	
	class LabicPCL {
		
	private:
		boost::thread m_Thread;
		Kinect *kinect;
        int width;
        int height;
        bool stop;
        void generateDepthCloud(uint16_t *depth);
		
	public:
        pcl::visualization::PCLVisualizer viewer;
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::PointCloud<pcl::PointXYZRGB> liveCloud;
        int viewPort;
        
        LabicPCL(Kinect *_kinect, bool& _stop, int _width, int _height);
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

#endif /* __LabicPCL__ */
