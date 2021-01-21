#include "structureFromMotion.h"
#include "bundleAdjustment.h"


#define RATIO_THRESH 0.7
#define CAMERA_SCALE 0.4

// builds pcl and reconstract camera position
// gets camera calib and vector of features
structureFromMotion::structureFromMotion(cameraCalibration& calib, std::vector<imageFeatures>& features, bool optimization) : _pclPointCloudPtrPreviousSize(0), _cameraMeshesPreviousSize(0)
{
    // initialize the pcl point cloud ptr which contains points - each point has XYZ coordinates and also has a Red Green Blue params - RGB
    _pclPointCloudPtr.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    cv::Mat E, rotation, translation, mask, points4D;

    std::vector<cv::Point2f> currentKeyPoints, otherKeyPoints;
    features::getCurrentKeyPoints(currentKeyPoints ,0);
    features::getOtherKeyPoints(otherKeyPoints, 0);

    // getting essential metrix and recover first camera position
    E = findEssentialMat(currentKeyPoints, otherKeyPoints, calib.getFocal(), calib.getPP(), cv::RANSAC, 0.9, 3.0, mask);
    cv::recoverPose(E, currentKeyPoints, otherKeyPoints, rotation, translation, calib.getFocal(), calib.getPP(), mask);

    cv::Mat firstProjection, zeros = (cv::Mat_<double>(3, 1) << 0, 0, 0);
    hconcat(calib.getCameraMatrix(), zeros, firstProjection);
    features[0].projection = firstProjection;

    addCameraToVisualizer((cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1), zeros); // first camera position is without rotation nor translation
    features[0].rotation = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    features[0].translation = zeros;

    std::vector<pointInCloud> previous3dPoints;
    std::thread mythread([this] { showPointCloud(); }); // thread of showPointCloud - LIVE point cloud update in the viewer

    for (unsigned int i = 1; i < features.size(); i++) // go throw all iamges features
    {
        features::getCurrentKeyPoints(currentKeyPoints, i - 1);
        features::getOtherKeyPoints(otherKeyPoints, i - 1);

        if (i > 1) // first pair will skip this
        {
            cv::Mat descriptors3d;

            for(int j = 0; j < previous3dPoints.size(); j++)
            { // descriptors at pointInCloud position
                descriptors3d.push_back(features[i - 1].descriptors.row(previous3dPoints[j].otherKeyPointsIdx));
            }

            std::vector<cv::Point2d> imagePoints;
            std::vector<int> newObjIndexes;

            obtainMatches(features[i], descriptors3d, imagePoints, newObjIndexes, optimization);

            std::vector<cv::Point3d> objectPoints;

            for (int index : newObjIndexes)
            {
                objectPoints.push_back(previous3dPoints[index].point);
            }
            cv::solvePnPRansac(objectPoints, imagePoints, calib.getCameraMatrix(), calib.getDistortionCoefficients(), rotation, translation, false, 300, 3.0);
            //TO DO FIX - 19 error OpenCV(4.4.0-dev) Error: Unspecified error (> DLT algorithm needs at least 6 points for pose estimation from 3D-2D point correspondences. (expected: 'count >= 6'), where 'count' is 5 must be greater than or equal to '6' is 6) in void __cdecl cvFindExtrinsicCameraParams2(const struct CvMat*, const struct CvMat*, const struct CvMat*, const struct CvMat*, struct CvMat*, struct CvMat*, int), file C : \cv\opencv - master\modules\calib3d\src\calibration.cpp, line 1173

            // Convert from Rodrigues format to rotation matrix and build the 3x4 Transformation matrix
            cv::Rodrigues(rotation, rotation);

            previous3dPoints.clear();
        }
        addCameraToVisualizer(rotation, translation);
        features[i].rotation = rotation;
        features[i].translation = translation;

        // triangulate the points
        hconcat(rotation, translation, translation);
        features[i].projection = calib.getCameraMatrix() * translation;
        triangulatePoints(features[i - 1].projection, features[i].projection, currentKeyPoints, otherKeyPoints, points4D);
       

        for (int j = 0; j < points4D.cols; j++)
        {
            pointInCloud point;

            // convert point from homogeneous to 3d cartesian coordinates
            point.point = cv::Point3d(points4D.at<float>(0, j) / points4D.at<float>(3, j), points4D.at<float>(1, j) / points4D.at<float>(3, j), points4D.at<float>(2, j) / points4D.at<float>(3, j));


            point.otherKeyPointsIdx =  features[i - 1].matchingKeyPoints.otherKeyPointsIdx[j];
            point.imageIndex = i;

            cv::Vec3b color = features[i - 1].image.at<cv::Vec3b>(cv::Point(currentKeyPoints[j].x, currentKeyPoints[j].y));

            // color.val[2] - red, color.val[1] - green, color.val[0] - blue
            uint32_t rgb = (color.val[2] << 16 | color.val[1] << 8 | color.val[0]);
            point.color = *reinterpret_cast<float*>(&rgb);
            // (x, y, z, r, g, b)
            pcl::PointXYZRGB pclPoint(point.point.x, point.point.y, point.point.z, color.val[2], color.val[1], color.val[0]);

            _pointCloud.push_back(point);
            previous3dPoints.push_back(point);
            _pclPointCloudPtr->push_back(pclPoint);
        }
        cout << "Points cloud " << i - 1 << " / " << i << endl;

    }
    cout << "Number of points in the point cloud " << _pointCloud.size() << endl;
    cout << "Starting Bundle Adjustment" << endl;

    bundleAdjustment::bundleAdjustment(_pointCloud, *_pclPointCloudPtr, features, calib);
    _pclPointCloudPtrPreviousSize++; // force an updated of the cloud in the point cloud viewer
    _cameraMeshesPreviousSize++;
    mythread.join();
}

// obtain matches
void obtainMatches(imageFeatures features, cv::Mat& otherdescriptors, std::vector<cv::Point2d>& outputPoints2d, std::vector<int>& outputPoints2dIdx, bool optimization)
{
    std::vector<std::vector<cv::DMatch>> matches;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (optimization)
    {
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }
    else
    {
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
    }

    matcher->knnMatch(otherdescriptors, features.descriptors, matches, 2);

    for (std::vector<cv::DMatch>& matche : matches)
    {
        if (matche[0].distance < RATIO_THRESH * matche[1].distance)
        {
            outputPoints2d.push_back(features.keyPoints[matche[0].trainIdx].pt);
            outputPoints2dIdx.push_back(matche[0].queryIdx);
        }
    }
}

void structureFromMotion::showPointCloud()
{
    // initialize the pcl 3D Viewer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    // display the point cloud
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setShowFPS(true);
    viewer->setWindowName("Point Cloud Viewer");
    viewer->initCameraParameters();
    viewer->resetCamera();

    while (!viewer->wasStopped())
    {
        if (_pclPointCloudPtrPreviousSize != _pclPointCloudPtr->size())
        {
            viewer->removeAllPointClouds();
            viewer->addPointCloud<pcl::PointXYZRGB>(_pclPointCloudPtr, "point cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point cloud");
            _pclPointCloudPtrPreviousSize = _pclPointCloudPtr->size();
        }
        if (_cameraMeshesPreviousSize != _cameraMeshes.size()) // an overkill, should add each camera once
        {
            viewer->removeAllShapes();
            int id = 0;
            for(const pcl::PolygonMesh cam_mesh : _cameraMeshes)
            {
                viewer->removePolygonMesh("polygon" + std::to_string(id));
                viewer->addPolygonMesh(cam_mesh, "polygon" + std::to_string(id++));
            }
            for(const std::pair<pcl::PointXYZRGB, pcl::PointXYZRGB> oneline : _cameraLines)
            {
                viewer->addLine(oneline.first, oneline.second, "line" + std::to_string(id++));
            }
            _cameraMeshesPreviousSize = _cameraMeshes.size();
        }
        viewer->spinOnce(100);
    }
}

void structureFromMotion::addCameraToVisualizer(const cv::Mat rotation, const cv::Mat translation)
{
    cv::Mat t = -rotation.t() * translation;
    cv::Mat right = rotation.row(0).t() * CAMERA_SCALE;
    cv::Mat up = -rotation.row(1).t() * CAMERA_SCALE;
    cv::Mat forward = rotation.row(2).t() * CAMERA_SCALE;

    pcl::RGB rgb(255, 50, 50);

    pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
    pointCloud.push_back(toPointXYZRGB(t, rgb));
    pointCloud.push_back(toPointXYZRGB(t + forward + right / 1.5 + up / 2.0, rgb));
    pointCloud.push_back(toPointXYZRGB(t + forward + right / 1.5 - up / 2.0, rgb));
    pointCloud.push_back(toPointXYZRGB(t + forward - right / 1.5 + up / 2.0, rgb));
    pointCloud.push_back(toPointXYZRGB(t + forward - right / 1.5 - up / 2.0, rgb));

    int pyramidPolygon[6][3] = { {0,1,2}, {0,3,1}, {0,4,3}, {0,2,4}, {3,1,4}, {2,4,1} };
    
    pcl::PolygonMesh polygonMesh;
    polygonMesh.polygons.resize(6);
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            polygonMesh.polygons[i].vertices.push_back(pyramidPolygon[i][j]);
        }
    }
    pcl::toPCLPointCloud2(pointCloud, polygonMesh.cloud);

    _cameraMeshes.push_back(polygonMesh);
    _cameraLines.push_back(std::make_pair(toPointXYZRGB(t, rgb), toPointXYZRGB(t + forward * 2.0, rgb)));
}

pcl::PointXYZRGB toPointXYZRGB(cv::Mat point3d, pcl::RGB rgb)
{
    return pcl::PointXYZRGB(point3d.at<double>(0), point3d.at<double>(1), point3d.at<double>(2), rgb.r, rgb.b, rgb.g);
}

// saves the Point cloud to file
void structureFromMotion::savePointCloud(std::string filePath)
{
    pcl::io::savePLYFileBinary(filePath, *_pclPointCloudPtr);
}
