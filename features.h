#pragma once
#include "cameraCalibration.h"

#include "opencv2/features2d.hpp"

#include <set>

struct matchingKeyPoints
{
	std::vector<cv::Point2f> currentKeyPoints;
	std::vector<cv::Point2f> otherKeyPoints;
	std::vector<int> currentKeyPointsIdx;
	std::vector<int> otherKeyPointsIdx;	
}typedef matchingKeyPoints;

//represent interesting points in a image
struct imageFeatures
{
	std::string path;
	cv::Mat image;
	std::vector<cv::KeyPoint> keyPoints;
	cv::Mat descriptors;

	matchingKeyPoints matchingKeyPoints;

	cv::Mat projection;

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

// function declaration
int getScreenWidth();
void resizeWithAspectRatio(cv::Mat& image, int width);
