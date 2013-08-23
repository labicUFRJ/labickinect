//
//  pcl.h
//  LabicKinect
//
//  Created by Mario Cecchi on 05/07/13.
//  Copyright (c) 2013 Mario Cecchi. All rights reserved.
//


#include <math.h>
#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <iterator>
#include <vector>
#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "opencv2/core/core.hpp"

#ifndef __LabicKinect__
#include "LabicKinect.h"
#endif

void show_pcl();


namespace Labic {
	
	class LabicPCL {
		
	private:
		Kinect *kinect;
		
	public:
        // point cloud and viewer
        pcl::visualization::PCLVisualizer         viewer;
        pcl::PointCloud<pcl::PointXYZRGB>         cloud;
        int viewPort;
        
        LabicPCL(Kinect *_kinect);
		void start();
		void join();
        
        void addCameras(const std::vector<cv::Mat>&         T,
                                   const std::vector<cv::Mat>&         R);
        
        void updateCloud(std::vector< cv::Point3d >& objPoints,
                                               cv::Mat img);
        
        void display();
        
    };

    
}