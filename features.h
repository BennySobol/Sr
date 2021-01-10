#pragma once
#include "cameraCalibration.h"

#include "opencv2/features2d.hpp"

struct matchingKeyPoints
{
	std::vector<int> currentKeyPointsIdx;
	std::vector<int> otherKeyPointsIdx;	
} typedef matchingKeyPoints;

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
	static std::vector<imageFeatures> _features;
public:
	void matchFeatures(cameraCalibration calib, bool optimization=true, bool showMatchingFeature=false);
	features(std::vector<std::string> images);
	features(std::string filePath);
	std::vector<imageFeatures>& getFeatures();

	static void getCurrentKeyPoints(std::vector<cv::Point2f>& currentKeyPoints, int featureIndex);
	static void getOtherKeyPoints(std::vector<cv::Point2f>& otherKeyPoints, int featureIndex);

	void save(std::string filePath);
	void load(std::string filePath);
};

// function declaration
int getScreenWidth();
void resizeWithAspectRatio(cv::Mat& image, int width);
