#pragma once
#include "features.h"


#include <opencv2/calib3d.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <thread>


// point part of PCL contains 3D point and its 2D origin point and color
struct CloudPoint {
	cv::Point3d point;
	std::vector<int>index_of_2d_origin;
	int seen = 0; // how many cameras have seen this point
	float color;
} typedef CloudPoint;

class cameraPosition
{
private:
	pcl::PointCloud<pcl::PointXYZ> pointCloud;// points of xyz - 3D points

public:
	cameraPosition(cameraCalibration calib, std::vector<imageFeatures> _features);
};
