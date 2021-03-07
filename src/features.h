#pragma once
#include "cameraCalibration.h"

#include <pcl/registration/transforms.h>

#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/cuthill_mckee_ordering.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/bandwidth.hpp>

#include "opencv2/features2d.hpp"


// function declaration
int getScreenWidth();
void resizeWithAspectRatio(cv::Mat& image, int width);
std::string getFileNameWithExtension(std::string path);
std::string getFileName(std::string path);


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
private:
	inline static cv::Mat smallExtractFeatures(std::string path, int size);
	inline static int widthFor900Features(std::string path);
	inline static imageFeatures extractFeatures(std::string path, double downScale = 1);
	static int matchFeaturesScore(cv::Mat firstImageDescriptors, cv::Mat secondImageDescriptors);
	static void matchFeatures(imageFeatures& firstImageFeatures, imageFeatures& secondImageFeatures,
		std::vector<cv::Point2f>& firstPoints, std::vector<cv::Point2f>& secondPoints);
	static std::vector<int> findMaxRoute(const std::vector<std::vector<int>> scoreMatrix);
	static cv::Ptr<cv::FeatureDetector> _detector;
	static cv::Ptr<cv::DescriptorMatcher> _matcher;
public:
	void matchAllFeatures(cameraCalibration calib, bool showMatchingFeature = false);
	features(std::vector<std::string> images, double downScaleFactor = 1);
	std::vector<imageFeatures>& getFeatures();

	static void sortImages(std::vector<std::string>& images);
	static void getCurrentKeyPoints(std::vector<cv::Point2f>& currentKeyPoints, int featureIndex);
	static void getOtherKeyPoints(std::vector<cv::Point2f>& otherKeyPoints, int featureIndex);
};

int features::widthFor900Features(std::string path)
{
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat src = cv::imread(path, cv::IMREAD_GRAYSCALE);
	cv::Mat save = src;
	int size = 600;
	while (keypoints.size() < 800)
	{
		if (size > src.size().width)
		{
			return  src.size().width;
		}
		resizeWithAspectRatio(save, size);
		_detector->detect(save, keypoints);
		save = src;
		size += 100;
	}
	return size;
}

cv::Mat features::smallExtractFeatures(std::string path, int size)
{
	cv::Mat src = cv::imread(path, cv::IMREAD_GRAYSCALE);

	resizeWithAspectRatio(src, size);

	if (!src.empty())
	{
		cv::Mat descriptors;
		std::vector<cv::KeyPoint> keypoints;
		_detector->detectAndCompute(src, cv::noArray(), keypoints, descriptors);
		std::cout << getFileNameWithExtension(path) << " " << keypoints.size() << " features were extracted\n";
		return descriptors;
	}
	throw std::runtime_error(path + " is not a valid image");
}

imageFeatures features::extractFeatures(std::string path, double downScale)
{
	cv::Mat src = cv::imread(path);
	cv::resize(src, src, src.size() / (int)downScale);
	if (!src.empty())
	{
		cv::Mat descriptors;
		std::vector<cv::KeyPoint> keypoints;
		_detector->detectAndCompute(src, cv::noArray(), keypoints, descriptors);
		std::cout << getFileNameWithExtension(path) << " " << keypoints.size() << " features were extracted\n";
		return imageFeatures{ path, src, keypoints, descriptors };
	}
	throw std::runtime_error(path + " is not a valid image");
}