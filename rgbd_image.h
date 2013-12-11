#ifndef __LABIC_RGBDIMAGE_H__
#define __LABIC_RGBDIMAGE_H__

#include "common.h"

namespace labic {
	class RGBDImage {
	public:
		RGBDImage();
		RGBDImage(std::vector<uint8_t>& _raw_rgb, std::vector<uint16_t>& _raw_depth, uint32_t _timestamp);
		//TODO copy constructor, move constructor
		//~RGBDImage();
		//RGBDImage(const RGBDImage& other);
		//RGBDImage(RGBDImage&& other);
		void update(std::vector<uint8_t>& _raw_rgb, std::vector<uint16_t>& _raw_depth, uint32_t _timestamp);
		void copyTo(RGBDImage& other) const;
		RGBDImage& operator=(const RGBDImage& other);
		bool operator==(const RGBDImage& other) const;

		const cv::Mat3b& rgb() const { return m_rgb; }
		const cv::Mat1f& depth() const { return m_depth; }
		const uint32_t timestamp() const { return time; }
		const Cloud pointCloud() const;
		const Cloud pointCloudOfSelection(std::vector<cv::Point2f> pts) const;
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
		constexpr static double fx_d = 1.0 / 5.9421434211923247e+02;
		constexpr static double fy_d = 1.0 / 5.9104053696870778e+02;
		constexpr static double cx_d = 3.3930780975300314e+02;
		constexpr static double cy_d = 2.4273913761751615e+02;
	};
}

#endif /* __LABIC_RGBDIMAGE_H__ */
