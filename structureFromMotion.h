#pragma once
#include "features.h"

#include <thread>
#include <opencv2/calib3d.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>

// point part of PCL contains 3D point and its 2D origin point and color
struct pointInCloud {
	int imageIndex;

	cv::Point3d point;
	int otherKeyPointsIdx;

	float color;
} typedef pointInCloud;

// camera motion and pointcloud reconstruction
class structureFromMotion
{
private:
	std::vector<pointInCloud> _pointCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pclPointCloudPtr;
	std::vector<pcl::PolygonMesh> _cameraMeshes;
	std::vector<std::pair<pcl::PointXYZRGB, pcl::PointXYZRGB>> _cameraLines;
	int _pclPointCloudPtrPreviousSize;
	int _cameraMeshesPreviousSize;

	void showPointCloud();
	void addCameraToVisualizer(const cv::Mat rotation, const cv::Mat translation);
public:
	structureFromMotion(cameraCalibration& calib, std::vector<imageFeatures>& _features, bool optimization=true);
	
	void savePointCloud(std::string filePath);
};

void obtainMatches(imageFeatures features, cv::Mat& otherdescriptors, std::vector<cv::Point2d>& outputPoints2d, std::vector<int>& outputPoints2dIdx, bool optimization=true);
inline pcl::PointXYZRGB toPointXYZRGB(cv::Mat point3d, pcl::RGB rgb);