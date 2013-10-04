#ifndef __LABIC_RGBDIMAGE_H__
#define __LABIC_RGBDIMAGE_H__

#include "common.h"

namespace labic {
	class RGBDImage {
	public:
		RGBDImage();
		RGBDImage(std::vector<uint8_t>& _raw_rgb, std::vector<uint16_t>& _raw_depth, uint32_t _timestamp);
		//~RGBDImage();
		void update(std::vector<uint8_t>& _raw_rgb, std::vector<uint16_t>& _raw_depth, uint32_t _timestamp);
		void copyTo(RGBDImage& other) const;
		RGBDImage& operator=(const RGBDImage& other);
		bool operator==(const RGBDImage& other) const;

		const cv::Mat3b& rgb() const { return m_rgb; }
		const cv::Mat1f& depth() const { return m_depth; }
		const uint32_t timestamp() const { return time; }
		const pcl::PointCloud<pcl::PointXYZRGB> pointCloud() const;
		const pcl::PointCloud<pcl::PointXYZRGB> pointCloudOfSelection(std::vector<cv::Point2f> pts) const;
		const pcl::PointXYZRGB point(int r, int c) const;
		const pcl::PointXYZRGB point(int i) const;
		bool rgbPixelHasDepth(int r, int c) const { return m_depth(r,c) > 1e-5; }
		bool rgbPixelHasDepth(int i) const { return rgbPixelHasDepth(i/width, i%width); }


	private:
		cv::Mat3b m_rgb;
		cv::Mat1f m_depth;
		std::vector<uint8_t> raw_rgb;
		std::vector<uint16_t> raw_depth;
		uint32_t time;
	};
}

#endif /* __LABIC_RGBDIMAGE_H__ */