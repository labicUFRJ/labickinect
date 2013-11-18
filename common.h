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
#include <boost/thread.hpp>
#include <pcl/common/common.h>
#include "opencv2/core/core.hpp"

namespace labic {
	static const int width		 = 640;
	static const int height		 = 480;
	static const int DEPTH_BLANK = 2047;

	inline int mmToRaw(float depthValue) {
		int z = DEPTH_BLANK;
		if (depthValue > 0) {
			z = (int) MAX(0, ((1.0/(depthValue/1000.0)) - 3.3309495161)/(-0.0030711016));
		}
		return z;
	}

	inline pcl::PointXYZ pointInPixelSpace(pcl::PointXYZRGB p) {
		static const double f = 580.0; // focal length of ir camera in pixels
		static const double b = 0.075; // 7.5 cm in m
		p.x /= 1000; // convert from mm scale to m
		p.y /= 1000;
		p.z /= 1000;

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


//	inline cv::Point3d ptToPoint3d(float cgx, float cgy, float cgz) {
//		double fx_d = 1.0 / 5.9421434211923247e+02;
//		double fy_d = 1.0 / 5.9104053696870778e+02;
//		double cx_d = 3.3930780975300314e+02;
//		double cy_d = 2.4273913761751615e+02;
//
//		cgz = cgz/1000.0; // cgz has to be in mm
//
//		if (cgz == 0 || cgz == DEPTH_BLANK) {
//			return cv::Point3d(0,0,0);
//		}
//
//		return cv::Point3d(
//						   (cgx - cx_d) * cgz * fx_d,
//						   (cgy - cy_d) * cgz * fy_d,
//						   cgz
//						   );
//	}
//
//
//	inline pcl::PointXYZRGB ptToPointXYZRGB(float cgx, float cgy, float cgz) {
//		double fx_d = 1.0 / 5.9421434211923247e+02;
//		double fy_d = 1.0 / 5.9104053696870778e+02;
//		double cx_d = 3.3930780975300314e+02;
//		double cy_d = 2.4273913761751615e+02;
//		pcl::PointXYZRGB pt;
//
//		cgz = cgz/1000.0; // cgz has to be in mm
//
//		if (cgz == 0 || cgz == DEPTH_BLANK) {
//			pt.x = pt.y = pt.z = 0;
//		} else {
//
//			pt.x = (cgx - cx_d) * cgz * fx_d;
//			pt.y = (cgy - cy_d) * cgz * fy_d;
//			pt.z = cgz;
//		}
//
//		return pt;
//	}
//
//
//	inline bool frameToPointCloud(const cv::Mat& rgb, const uint16_t* depth, pcl::PointCloud<pcl::PointXYZRGB>& _cloud, const std::vector<cv::Point2f> pts = std::vector<cv::Point2f>()) {
//		pcl::PointCloud<pcl::PointXYZRGB> cloud;
//		pcl::PointXYZRGB pt;
//		/*    uint16_t *depth;
//		 cv::Mat rgb(cv::Size(width, height), CV_8UC3, cv::Scalar(0));*/
//		cv::Vec3b ptRGB;
//		unsigned int i;
//		unsigned int x, y;
//		clock_t t;
//
//		cloud.clear();
//		cloud.reserve(width*height);
//
//		t = clock();
//		// If no specific points were specified
//		if (pts.empty()) {
//			for (i=0; i<width*height; i++) {
//				y = i/width;
//				x = i%width;
//
//				// If point has no depth available, skip it
//				if (depth[i] > 0) {
//					pt = ptToPointXYZRGB(x, y, depth[i]);
//					ptRGB = rgb.at<cv::Vec3b>(y,x);
//					pt.r = ptRGB[2];
//					pt.g = ptRGB[1];
//					pt.b = ptRGB[0];
//
//					cloud.push_back(pt);
//				}
//			}
//		} else {
//			for (i=0; i<pts.size(); i++) {
//				y = pts[i].y;
//				x = pts[i].x;
//				unsigned int index = y*width + x;
//
//				// If point has no depth available, skip it
//				if (depth[index] > 0) {
//					pt = ptToPointXYZRGB(x, y, depth[index]);
//					ptRGB = rgb.at<cv::Vec3b>(y,x);
//					pt.r = ptRGB[2];
//					pt.g = ptRGB[1];
//					pt.b = ptRGB[0];
//
//					cloud.push_back(pt);
//				}
//			}
//		}
//
//		t = clock() - t;
//
//		_cloud = cloud;
//		//free(depth);
//
//		//    std::cout << "frameToPointCloud time: " << 1000*((float)t)/CLOCKS_PER_SEC << " ms " << std::endl;
//
//		return true;
//
//	}
	
}

#endif /* __LABICKINECT_COMMON_H__ */
