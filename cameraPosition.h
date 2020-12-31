#pragma once
#include "features.h"


#include <opencv2/calib3d.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <thread>


// point part of PCL contains 3D point and its 2D origin point and color
struct CloudPoint {
	cv::Point3d point;
	std::vector<int> indexOf2dOrigin;
	float color;
} typedef CloudPoint;

class cameraPosition
{
private:
	std::string _imagesLocation;
	std::vector<CloudPoint> _pointCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pclPointCloudPtr;

public:
	cameraPosition(cameraCalibration calib, std::vector<imageFeatures> _features, std::string imagesLocation);
	void showPointCloud();
	void savePointCloud();
};

//NOTE: COMMENT BOTH PROGRAMMERS!
void obtainMatches(std::vector<cv::KeyPoint>& kp1, std::vector<cv::KeyPoint>& kp2, cv::Mat& descriptors1, cv::Mat& descriptors2, std::vector<cv::Point2d>& points1, std::vector<cv::Point2d>& points2, std::vector<int>& points1_idx, std::vector<int>& points2_idx);
