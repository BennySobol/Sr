#pragma once
#include "features.h"
#include "visualizer.h"

#include <opencv2/calib3d.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>

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
	std::vector<pointInCloud> _previous3dPoints;
	cv::Scalar _averageColor;
	visualizer* _visualizer;

	void obtainMatches(imageFeatures features, cv::Mat descriptors3d, std::vector<cv::Point2d>& imagePoints2d, std::vector<cv::Point3d>& objectPoints3d, std::vector<pointInCloud> previous3dPoints);
	void addPoints4DToPointCloud(cv::Mat points4D, imageFeatures feature, int index, std::vector<cv::Point2f> currentKeyPoints);
public:
	structureFromMotion(cameraCalibration& calib, std::vector<imageFeatures>& _features, double cameraScale = 0.4);
	~structureFromMotion()
	{
		delete _visualizer;
	}
	void savePointCloud(std::string filePath);
};
