#include "rgbd_image.h"
#include "opencv2/imgproc/imgproc.hpp"

using namespace labic;

RGBDImage::RGBDImage() : m_rgb(height, width), m_depth(height, width), raw_rgb(0), raw_depth(0), time(0) {
}

//RGBDImage::~RGBDImage() {
//	delete &raw_rgb;
//	delete &raw_depth;
//	delete &m_rgb;
//	delete &m_depth;
//	delete &timestamp;
//}

RGBDImage::RGBDImage(std::vector<uint8_t>& _raw_rgb, std::vector<uint16_t>& _raw_depth, uint32_t _timestamp)
: m_rgb(height, width), m_depth(height, width), raw_rgb(_raw_rgb.size()), raw_depth(_raw_depth.size()), time(_timestamp) {
	update(_raw_rgb, _raw_depth, _timestamp);
}

void RGBDImage::update(std::vector<uint8_t>& _raw_rgb, std::vector<uint16_t>& _raw_depth, uint32_t _timestamp) {
	if (_raw_rgb.size() != raw_rgb.size() || _raw_depth.size() != raw_depth.size()) {
		raw_rgb.resize(_raw_rgb.size());
		raw_depth.resize(_raw_depth.size());
	}

	time = _timestamp;
	raw_rgb.swap(_raw_rgb);
	raw_depth.swap(_raw_depth);

	for (int i=0; i<width*height; i++) {
		int x = i % width;
		int y = i / width;

		m_depth(y,x) = raw_depth[i];
		m_rgb(y,x)[0] = raw_rgb[3*i+2];
		m_rgb(y,x)[1] = raw_rgb[3*i+1];
		m_rgb(y,x)[2] = raw_rgb[3*i+0];
	}
}

void RGBDImage::copyTo(RGBDImage& other) const {
	m_depth.copyTo(other.m_depth);
	m_rgb.copyTo(other.m_rgb);
	other.raw_rgb = raw_rgb;
	other.raw_depth = raw_depth;
	other.time = time;
}

RGBDImage& RGBDImage::operator=(const RGBDImage& other) {
	other.copyTo(*this);
	return *this;
}

bool RGBDImage::operator==(const RGBDImage& other) const {
	return other.time == time;
}

const pcl::PointXYZRGB RGBDImage::point(int r, int c) const {
	pcl::PointXYZRGB pt;

	float d = m_depth(r,c); // depth has to be in mm

	pt.x = (c - cx_d) * d * fx_d;
	pt.y = (r - cy_d) * d * fy_d;
	pt.z = d;
	pt.r = m_rgb(r,c)[2];
	pt.g = m_rgb(r,c)[1];
	pt.b = m_rgb(r,c)[0];

	return pt;
}

const pcl::PointXYZRGB RGBDImage::point(int i) const {
	return point(i/width, i%width);
}

const pcl::PointCloud<pcl::PointXYZRGB> RGBDImage::pointCloud() const {
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	std::vector<cv::Point2f> pts(0);

	//cloud.reserve(width*height);

	for (int i=0; i<width*height; i++) {
		if (rgbPixelHasDepth(i)) cloud.push_back(point(i));
	}

	return cloud;
}

const pcl::PointCloud<pcl::PointXYZRGB> RGBDImage::pointCloudOfSelection(std::vector<cv::Point2f> pts) const {
	pcl::PointCloud<pcl::PointXYZRGB> cloud;

	//cloud.reserve(pts.size());

	for (unsigned int i=0; i<pts.size(); i++) {
		if (rgbPixelHasDepth(pts[i].y,pts[i].x)) cloud.push_back(point(pts[i].y,pts[i].x));
	}

	return cloud;
}
