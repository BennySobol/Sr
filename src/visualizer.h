#pragma once
#include "features.h"

#include <thread>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>


class visualizer
{
private:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pclPointCloudPtr;
	std::vector<imageFeatures>& _features;
	cameraCalibration& _calib;
	double _cameraScale;
	bool _isUpdateRequired;
	int _curentCameraIdx;
	void addCamerasToViewer(pcl::visualization::PCLVisualizer::Ptr& viewer);
	pcl::PointXYZ toPointXYZ(cv::Mat point3d);
public:
	visualizer(cameraCalibration& calib, std::vector<imageFeatures>& features, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pclPointCloudPtr, double cameraScale);
	void show();
	void update();
	void updateAll();
};
