#pragma once

#include <vector>
#include <set>
#include <iostream>

#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <map>

struct matchingFeatures
{
	std::vector<cv::Point2f> currentKeyPoints;
	std::vector<cv::Point2f> otherKeyPoints;
};

struct imageFeatures
{
	std::string path;
	cv::Mat image;
	std::vector<cv::KeyPoint> keyPoints;
	cv::Mat descriptors;

	std::map<int, matchingFeatures> matchesKeyPoints; // path to other images, <source keyPoints, other images keyPoints>

} typedef imageFeatures;

class features
{
protected:
	std::vector<imageFeatures> _features;
public:
	void matchFeatures();
	features(std::vector<std::string> images);
	std::vector<imageFeatures>& getFeatures();
};
