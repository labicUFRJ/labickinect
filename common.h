//
//  common.h
//  LabicKinect
//
//  Created by Mário Cecchi on 27/09/13.
//
//

#ifndef __LABICKINECT_COMMON_H__
#define __LABICKINECT_COMMON_H__

#include <iostream>
#include <vector>
#include <chrono>
#include <boost/thread.hpp>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include "opencv2/core/core.hpp"
#include "tools/debug.h"
#include "reconstructor/world.h"

namespace labic {
	static const int width		 = 640;
	static const int height		 = 480;
	static const int DEPTH_BLANK = 2047;

	typedef std::chrono::high_resolution_clock hrclock;
	typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;

	inline double diffTime(hrclock::time_point a, hrclock::time_point b) {
		return std::chrono::duration_cast<std::chrono::milliseconds>(a - b).count()/1000.0;
	}

	inline int diffTimeMs(hrclock::time_point a, hrclock::time_point b) {
		return std::chrono::duration_cast<std::chrono::milliseconds>(a - b).count();
	}

	inline int mmToRaw(float depthValue) {
		int z = DEPTH_BLANK;
		if (depthValue > 0) {
			z = (int) MAX(0, ((1.0/(depthValue)) - 3.3309495161)/(-0.0030711016));
		}
		return z;
	}

	inline pcl::PointXYZ pointInPixelSpace(pcl::PointXYZRGB p) {
		static const double f = 580.0; // focal length of ir camera in pixels
		static const double b = 0.075; // baseline: 7.5 cm in meters

		pcl::PointXYZ r;
		r.x = (f/p.z)*p.x + width/2;
		r.y = (f/p.z)*p.y + height/2;
		r.z = (f/p.z)*b;

		return r;
	}

	inline double pixelDistance(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2) {
		pcl::PointXYZ projP1 = pointInPixelSpace(p1);
		pcl::PointXYZ projP2 = pointInPixelSpace(p2);

		return sqrt(pow(projP1.x - projP2.x, 2) + pow(projP1.y - projP2.y, 2) + pow(projP1.z - projP2.z, 2));
	}

	inline double squaredPixelDistance(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2) {
		pcl::PointXYZ projP1 = pointInPixelSpace(p1);
		pcl::PointXYZ projP2 = pointInPixelSpace(p2);

		return (pow(projP1.x - projP2.x, 2) + pow(projP1.y - projP2.y, 2) + pow(projP1.z - projP2.z, 2));
	}

	template <class T>
	pcl::PointCloud<T> downsample(const pcl::PointCloud<T>& cloud, double leafSize = 0.01) {
		pcl::PointCloud<T> cloud_filtered;

//		std::cout << "PointCloud before filtering: " << cloud.width * cloud.height << " data points (" << pcl::getFieldsList(cloud) << ")." << std::endl;

		// Create the filtering object
		pcl::VoxelGrid<T> sor;
		sor.setInputCloud(cloud.makeShared());
		sor.setLeafSize(leafSize, leafSize, leafSize);
		sor.filter(cloud_filtered);

//		std::cout << "PointCloud after filtering: " << cloud_filtered.width * cloud_filtered.height << " data points (" << pcl::getFieldsList (cloud_filtered) << ")." << std::endl;

		return cloud_filtered;
	}
}

#endif /* __LABICKINECT_COMMON_H__ */
