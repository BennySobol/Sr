#pragma once

#include <vector>
#include <set>
#include <iostream>

#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include "cameraCalibration.h"

struct matchingKeyPoints
{
	std::vector<cv::Point2f> currentKeyPoints;
	std::vector<cv::Point2f> otherKeyPoints;
	std::vector<int> currentKeyPointsIdx;
	std::vector<int> otherKeyPointsIdx;	
};

struct imageFeatures
{
	std::string path;
	cv::Mat image;
	std::vector<cv::KeyPoint> keyPoints;
	cv::Mat descriptors;

	matchingKeyPoints matchingKeyPoints;

} typedef imageFeatures;

class features
{
protected:
	std::vector<imageFeatures> _features;
public:
	void matchFeatures(cameraCalibration calib);
	features(std::vector<std::string> images);
	std::vector<imageFeatures>& getFeatures();
};
