#pragma once
#include "features.h"

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/calib3d.hpp>

class cameraPosition
{
private:
   pcl::PointCloud<pcl::PointXYZ> pointCloud;

public:
	cameraPosition(cameraCalibration calib, std::vector<imageFeatures> _features);

};
