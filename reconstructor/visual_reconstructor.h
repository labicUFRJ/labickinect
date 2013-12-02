#ifndef VISUAL_RECONSTRUCTOR_H_
#define VISUAL_RECONSTRUCTOR_H_

#include "../common.h"
#include "../rgbd_image.h"
#include "opencv2/features2d/features2d.hpp"
#include "ransac_aligner.h"

class VisualReconstructor {
public:
	VisualReconstructor();
	void operator () (labic::RGBDImage& rgbdPrevious, labic::RGBDImage& rgbdCurrent);
	const Eigen::Matrix4d& getFinalTransform() const { return transform; }
	const std::vector<int>& getFinalInliers() const { return transformationInliersIndexes; }
	double getFinalError() const { return ransacError; }

private:
	void extractRGBFeatures(const labic::RGBDImage& rgbd, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);

	void matchFeatures(std::vector<cv::KeyPoint>&   _keypoints_q,
					   const cv::Mat&               _descriptors_q,
					   std::vector<cv::KeyPoint>&   _keypoints_t,
					   const cv::Mat&               _descriptors_t,
					   std::vector<cv::DMatch>&     _matches);


	unsigned minFeatures;
	unsigned maxFeatures;
	unsigned maxDetectionIte;
	unsigned minInliers;
	double maxMatchDistance;
	double ransacError;

	cv::Ptr<cv::FastAdjuster>         adjuster;
	cv::Ptr<cv::DescriptorMatcher>    matcher;
	cv::Ptr<cv::DescriptorExtractor>  extractor;
	cv::Mat							  descriptorsPrevious;
	std::vector<cv::KeyPoint>		  featuresPrevious;
	uint32_t						  cachedPrevious;

	labic::RANSACAligner ransac;

    std::vector<int> transformationInliersIndexes;
	Eigen::Matrix4d transform;

};


#endif /* VISUAL_RECONSTRUCTOR_H_ */
