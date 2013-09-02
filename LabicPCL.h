//
//  pcl.h
//  LabicKinect
//
//  Created by Mario Cecchi on 05/07/13.
//  Copyright (c) 2013 Mario Cecchi. All rights reserved.
//


#include <iostream>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "opencv2/core/core.hpp"

#ifndef BOOST_THREAD_INCLUDED
#define BOOST_THREAD_INCLUDED
#include <boost/thread.hpp>
#endif

#ifndef __LabicKinect__
#include "LabicKinect.h"
#endif

void show_pcl();


namespace Labic {
	
	class LabicPCL {
		
	private:
		boost::thread m_Thread;
		Kinect *kinect;
        int width;
        int height;
        static const int REFRESH_INTERVAL = 1;
        void generateDepthCloud(uint16_t *depth);
		
	public:
        pcl::visualization::PCLVisualizer viewer;
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::PointCloud<pcl::PointXYZRGB> liveCloud;
        int viewPort;
        
        LabicPCL(Kinect *_kinect, int _width, int _height);
		void start();
		void join();
        
        void addCameras(const std::vector<cv::Mat>&         T,
                        const std::vector<cv::Mat>&         R);
        
        void updateCloud(std::vector< cv::Point3d >& objPoints, cv::Mat img);
        
        void display();
        
    };

    
}