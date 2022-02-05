#include "visualizer.h"
#include "loadImages.h"
#include <mutex>

std::mutex mutex;

visualizer::visualizer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pclPointCloudPtr, double cameraScale) : _pclPointCloudPtr(pclPointCloudPtr), _cameraScale(cameraScale), _isUpdateRequired(false), _curentCameraIdx(0)
{
	std::thread mythread([this] { show(); }); // thread of show - LIVE point cloud update in the viewer
	mythread.detach();
}

visualizer::~visualizer()
{
	_pclPointCloudPtr->clear();
}

void visualizer::show()
{
	// initialize the pcl 3D Viewer
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	// display the point cloud
	viewer->setBackgroundColor(0.04, 0.04, 0.12);
	viewer->setShowFPS(true);
	viewer->setWindowName("Point Cloud Viewer");
	// pos_x, pos_y, pos_z, view_x, view_y, view_z
	viewer->setCameraPosition(0.100329, -2.76665, -6.4343, 0.0162269, -0.967499, 0.252354);

	while (!viewer->wasStopped())
	{
		if (_isUpdateRequired)
		{
			mutex.lock();
			viewer->removeAllPointClouds();
			viewer->addPointCloud<pcl::PointXYZRGB>(_pclPointCloudPtr, "point cloud");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point cloud");

			viewer->removeAllShapes();
			addCamerasToViewer(viewer);
			_isUpdateRequired = false;
			mutex.unlock();
		}
		viewer->spinOnce(100);
	}
}

void visualizer::addCamerasToViewer(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
	std::vector<imageFeatures>& features = features::getFeatures();
	for (int i = 0; i < features.size(); i++)
	{
		if (features[i].rotation.rows == 0) // if not reconstct yet
		{
			break;
		}
		_curentCameraIdx++;

		/* center formula from  https://colmap.github.io/format.html#images-txt
		The coordinates of the projection/camera center are given by -R^t * T,
		where R^t is the inverse/transpose of the 3x3 rotation matrix composed from the quaternion and T is the translation vector */
		cv::Mat center = -features[i].rotation.t() * features[i].translation;
		// formulas from http://www.pcl-users.org/How-to-draw-camera-pyramids-in-Visualizer-tp4019124p4021310.html
		cv::Mat right = features[i].rotation.row(0).t() * _cameraScale;
		cv::Mat up = -features[i].rotation.row(1).t() * _cameraScale;
		cv::Mat forward = features[i].rotation.row(2).t() * _cameraScale;

		pcl::PointXYZ middle, rightUp, rightDown, leftUp, leftDown;
		middle = toPointXYZ(center);
		rightUp = toPointXYZ(center + forward + right / 1.5 + up / 2.0);
		rightDown = toPointXYZ(center + forward + right / 1.5 - up / 2.0);
		leftUp = toPointXYZ(center + forward - right / 1.5 + up / 2.0);
		leftDown = toPointXYZ(center + forward - right / 1.5 - up / 2.0);

		// lines idea from function showCameras at https://github.com/PointCloudLibrary/pcl/blob/master/gpu/kinfu_large_scale/tools/standalone_texture_mapping.cpp
		std::string cameraName = getFileName(features[i].path);
		viewer->removeText3D(cameraName + std::to_string(i));
		viewer->addText3D(cameraName, middle, 0.1, 1.0, 1.0, 1.0, cameraName + std::to_string(i));

		viewer->addLine(middle, rightUp, 1, 1, 1, "camera" + std::to_string(i) + "line1");
		viewer->addLine(middle, rightDown, 1, 1, 1, "camera" + std::to_string(i) + "line2");
		viewer->addLine(middle, leftUp, 1, 1, 1, "camera" + std::to_string(i) + "line3");
		viewer->addLine(middle, leftDown, 1, 1, 1, "camera" + std::to_string(i) + "line4");
		viewer->addLine(rightDown, rightUp, 1, 1, 1, "camera" + std::to_string(i) + "line5");
		viewer->addLine(leftDown, leftUp, 1, 1, 1, "camera" + std::to_string(i) + "line6");
		viewer->addLine(leftUp, rightUp, 1, 1, 1, "camera" + std::to_string(i) + "line7");
		viewer->addLine(leftDown, rightDown, 1, 1, 1, "camera" + std::to_string(i) + "line8");
	}
}

// convert opencv Mat to pcl PointXYZ and returns it
pcl::PointXYZ visualizer::toPointXYZ(cv::Mat point3d)
{
	return pcl::PointXYZ(point3d.at<double>(0), point3d.at<double>(1), point3d.at<double>(2));
}

// this function will cause the thread to add new cameras to the viewer
void visualizer::update()
{
	_isUpdateRequired = true;
};

// this function will reset _curentCameraIdx, and the thread will readd all cameras to the viewer
void visualizer::updateAll()
{
	_isUpdateRequired = true;
	_curentCameraIdx = 0;
};
