#pragma once
#include "features.h"

#include <thread>
#include <opencv2/calib3d.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>



// point part of PCL contains 3D point and its 2D origin point and color
struct pointInCloud {
	int i;
	int j;

	cv::Point3d point;
	int otherKeyPointsIdx;
	float color;
} typedef pointInCloud;

class cameraPosition
{
private:
	std::vector<pointInCloud> _pointCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pclPointCloudPtr;

	int _pclPointCloudPtrPreviousSize;
public:
	cameraPosition(cameraCalibration calib, std::vector<imageFeatures> _features, bool optimization=true);
	void showPointCloud();
};

void obtainMatches(imageFeatures features, cv::Mat& otherdescriptors, std::vector<cv::Point2d>& outputPoints2d, std::vector<int>& outputPoints2dIdx, bool optimization=true);
