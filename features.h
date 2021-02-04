#pragma once
#include "cameraCalibration.h"

#include <pcl/registration/transforms.h>

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
	cv::Mat rotation;
	cv::Mat translation;

	Eigen::Affine3f getCamPose()
	{
		Eigen::Affine3f cameraPose;
		//add translations
		cameraPose(0, 3) = translation.at<double>(0, 0); //TX
		cameraPose(1, 3) = translation.at<double>(1, 0); //TY
		cameraPose(2, 3) = translation.at<double>(2, 0); //TZ

		//add rotations
		cameraPose(0, 0) = rotation.at<double>(0, 0);
		cameraPose(0, 1) = rotation.at<double>(0, 1);
		cameraPose(0, 2) = rotation.at<double>(0, 2);

		cameraPose(1, 0) = rotation.at<double>(1, 0);
		cameraPose(1, 1) = rotation.at<double>(1, 1);
		cameraPose(1, 2) = rotation.at<double>(1, 2);

		cameraPose(2, 0) = rotation.at<double>(2, 0);
		cameraPose(2, 1) = rotation.at<double>(2, 1);
		cameraPose(2, 2) = rotation.at<double>(2, 2);

		cameraPose(3, 0) = 0.0;
		cameraPose(3, 1) = 0.0;
		cameraPose(3, 2) = 0.0;
		cameraPose(3, 3) = 1.0; //Scale
		return cameraPose;
	}
} typedef imageFeatures;

class features
{
protected:
	static std::vector<imageFeatures> _features;
public:
	void matchFeatures(cameraCalibration calib, bool showMatchingFeature = false);
	features(std::vector<std::string> images, int downScale);
	std::vector<imageFeatures>& getFeatures();

	static void getCurrentKeyPoints(std::vector<cv::Point2f>& currentKeyPoints, int featureIndex);
	static void getOtherKeyPoints(std::vector<cv::Point2f>& otherKeyPoints, int featureIndex);
};

// function declaration
int getScreenWidth();
void resizeWithAspectRatio(cv::Mat& image, int width);