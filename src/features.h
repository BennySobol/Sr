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

	// based on readCamPoseFile function at https://github.com/PointCloudLibrary/pcl/blob/master/gpu/kinfu_large_scale/tools/standalone_texture_mapping.cpp
	Eigen::Matrix4f getCamPose()
	{
		Eigen::Matrix4f cameraPose;
		cameraPose << rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2), translation.at<double>(0, 0)/*Tx*/,
					  rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2), translation.at<double>(1, 0)/*Ty*/,
					  rotation.at<double>(2, 0), rotation.at<double>(2, 1), rotation.at<double>(2, 2), translation.at<double>(2, 0)/*Tz*/,
					  0, 0, 0, 1/*Scale*/;

        return cameraPose;
	}
} typedef imageFeatures;

class features
{
protected:
	static std::vector<imageFeatures> _features;
public:
	void matchFeatures(cameraCalibration calib, bool showMatchingFeature = false);
	features(std::vector<std::string> images, int downScaleFactor = 2);
	std::vector<imageFeatures>& getFeatures();

	static void getCurrentKeyPoints(std::vector<cv::Point2f>& currentKeyPoints, int featureIndex);
	static void getOtherKeyPoints(std::vector<cv::Point2f>& otherKeyPoints, int featureIndex);
};

// function declaration
int getScreenWidth();
void resizeWithAspectRatio(cv::Mat& image, int width);