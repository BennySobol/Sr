#include "visualizer.h"
#include "loadImages.h"

visualizer::visualizer(cameraCalibration& calib, std::vector<imageFeatures>& features, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pclPointCloudPtr, double cameraScale) : _calib(calib), _features(features), _pclPointCloudPtr(pclPointCloudPtr), _cameraScale(cameraScale), _isUpdateRequired(false), _curentCameraIdx(0)
{
    std::thread mythread([this] { show(); }); // thread of show - LIVE point cloud update in the viewer
    mythread.detach();
}

void visualizer::show()
{
    // initialize the pcl 3D Viewer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    // display the point cloud
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setShowFPS(true);
    viewer->setWindowName("Point Cloud Viewer");
    viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
        if (_isUpdateRequired)
        {
            viewer->removeAllPointClouds();
            viewer->addPointCloud<pcl::PointXYZRGB>(_pclPointCloudPtr, "point cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point cloud");
            
            viewer->removeAllShapes();
            addCamerasToViewer(viewer);
            _isUpdateRequired = false;
        }
        viewer->spinOnce(100);
    }
}

void visualizer::addCamerasToViewer(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    for (int i = 0; i < _features.size(); i++)
    {
        if (_features[i].rotation.rows == 0) // if not reconstct yet
            continue;

        // Check if camera has been already added to the visualizer
        if (_curentCameraIdx < i)
        {
            continue;
        }
        _curentCameraIdx++;

        cv::Mat t = -_features[i].rotation.t() * _features[i].translation;
        cv::Mat right = _features[i].rotation.row(0).t() * _cameraScale;
        cv::Mat up = -_features[i].rotation.row(1).t() * _cameraScale;
        cv::Mat forward = _features[i].rotation.row(2).t() * _cameraScale;

        pcl::PointXYZ middle, rightUp, rightDown, leftUp, leftDown;
        middle = toPointXYZ(t);
        rightUp = toPointXYZ(t + forward + right / 1.5 + up / 2.0);
        rightDown = toPointXYZ(t + forward + right / 1.5 - up / 2.0);
        leftUp = toPointXYZ(t + forward - right / 1.5 + up / 2.0);
        leftDown = toPointXYZ(t + forward - right / 1.5 - up / 2.0);

        std::string cameraName = getFileName(_features[i].path);
        viewer->removeText3D(cameraName + std::to_string(i));
        viewer->addText3D(cameraName, middle, 0.1, 1.0, 1.0, 1.0, cameraName + std::to_string(i));

        viewer->addLine(middle, rightUp, "camera" + std::to_string(i) + "line1");
        viewer->addLine(middle, rightDown, "camera" + std::to_string(i) + "line2");
        viewer->addLine(middle, leftUp, "camera" + std::to_string(i) + "line3");
        viewer->addLine(middle, leftDown, "camera" + std::to_string(i) + "line4");
        viewer->addLine(rightDown, rightUp, "camera" + std::to_string(i) + "line5");
        viewer->addLine(leftDown, leftUp, "camera" + std::to_string(i) + "line6");
        viewer->addLine(leftUp, rightUp, "camera" + std::to_string(i) + "line7");
        viewer->addLine(leftDown, rightDown, "camera" + std::to_string(i) + "line8");
    }
}

pcl::PointXYZ visualizer::toPointXYZ(cv::Mat point3d)
{
    return pcl::PointXYZ(point3d.at<double>(0), point3d.at<double>(1), point3d.at<double>(2));
}

void visualizer::update()
{
    _isUpdateRequired = true;
};

void visualizer::updateAll()
{
    _isUpdateRequired = true;
    _curentCameraIdx = 0;
};
