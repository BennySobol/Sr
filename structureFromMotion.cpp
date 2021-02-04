#include "structureFromMotion.h"
#include "bundleAdjustment.h"


#define RATIO_THRESH 0.7

// builds pcl and reconstract camera position
// gets camera calib and vector of features
structureFromMotion::structureFromMotion(cameraCalibration& calib, std::vector<imageFeatures>& features, double cameraScale)
{
    // initialize the pcl point cloud ptr which contains points - each point has XYZ coordinates and also has a Red Green Blue params - RGB
    _pclPointCloudPtr.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    _visualizer = new visualizer(calib, features, _pclPointCloudPtr, cameraScale);

    cv::Mat E, rotation, translation, points4D;

    std::vector<cv::Point2f> currentKeyPoints, otherKeyPoints;
    features::getCurrentKeyPoints(currentKeyPoints ,0);
    features::getOtherKeyPoints(otherKeyPoints, 0);

    // getting essential metrix and recover second camera position
    E = findEssentialMat(currentKeyPoints, otherKeyPoints, calib.getCameraMatrix(), cv::RANSAC, 0.9999999999999999, 1.0);

    cv::recoverPose(E, currentKeyPoints, otherKeyPoints, calib.getCameraMatrix(), rotation, translation);

    cv::Mat firstProjection, firstTranslation = (cv::Mat_<double>(3, 1) << 0, 0, 0);
    hconcat(calib.getCameraMatrix(), firstTranslation, firstProjection);
    features[0].projection = firstProjection;

    features[0].rotation = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    features[0].translation = firstTranslation;
    _visualizer->update();

    _averageColor = 0;

    for (unsigned int i = 1; i < features.size(); i++) // go throw all iamges
    {
        features::getCurrentKeyPoints(currentKeyPoints, i - 1);
        features::getOtherKeyPoints(otherKeyPoints, i - 1);

        if (i > 1) // first pair will skip this
        { // recover rotation and translation between current image and previous image
            cv::Mat descriptors3d;

            // descriptors of previous image thet was added to the poincloud
            for(int j = 0; j < _previous3dPoints.size(); j++)
            { // add descriptors at pointInCloud position
                descriptors3d.push_back(features[i - 1].descriptors.row(_previous3dPoints[j].otherKeyPointsIdx));
            }

            std::vector<cv::Point2d> imagePoints2d;
            std::vector<int> newObjIndexes;
            std::vector<cv::Point3d> objectPoints3d;

            // obtain matches between previous 3d points and current image 2d points
            obtainMatches(features[i], descriptors3d, imagePoints2d, objectPoints3d, _previous3dPoints);

            double maxVal;
            cv::minMaxIdx(imagePoints2d, nullptr, &maxVal);
            cv::solvePnPRansac(objectPoints3d, imagePoints2d, calib.getCameraMatrix(), calib.getDistortionCoefficients(), rotation, translation, true, 4096, maxVal * 0.0023, 0.9999999999999999);

            // Convert from Rodrigues format to rotation matrix and build the 3x4 Transformation matrix
            cv::Rodrigues(rotation, rotation);

            _previous3dPoints.clear();
        }
        features[i].rotation = rotation;
        features[i].translation = translation;

        // triangulate the points
        hconcat(rotation, translation, translation);
        features[i].projection = calib.getCameraMatrix() * translation;
        triangulatePoints(features[i - 1].projection, features[i].projection, currentKeyPoints, otherKeyPoints, points4D);
       
        addPoints4DToPointCloud(points4D, features[i - 1], i, currentKeyPoints);

        cout << "Points cloud " << i - 1 << " / " << i << endl;
        _visualizer->update();
    }
    cout << "Number of points in the point cloud " << _pointCloud.size() << endl;
    cout << "Starting Bundle Adjustment" << endl;
    
    // save occluded.jpg - an image with the average color of all points in the point cloud, used later in the pipline for the tuxtureing of untuxturd part
    cv::Mat avrageColorImage(features[0].image.size().height, features[0].image.size().width, CV_8UC3, _averageColor);
    cv::imwrite(features[0].path.substr(0, features[0].path.find_last_of('\\') + 1) + "occluded.jpg", avrageColorImage);

    bundleAdjustment::bundleAdjustment(_pointCloud, *_pclPointCloudPtr, features, calib);
    _visualizer->updateAll();
}

void structureFromMotion::addPoints4DToPointCloud(cv::Mat points4D, imageFeatures feature, int index, std::vector<cv::Point2f> currentKeyPoints)
{
    for (int j = 0; j < points4D.cols; j++)
    {
        pointInCloud point;

        // convert point from homogeneous to 3d cartesian coordinates
        point.point = cv::Point3d(points4D.at<float>(0, j) / points4D.at<float>(3, j), points4D.at<float>(1, j) / points4D.at<float>(3, j), points4D.at<float>(2, j) / points4D.at<float>(3, j));

        point.otherKeyPointsIdx = feature.matchingKeyPoints.otherKeyPointsIdx[j];
        point.imageIndex = index;

        cv::Vec3b color = feature.image.at<cv::Vec3b>(cv::Point(currentKeyPoints[j].x, currentKeyPoints[j].y));

        // color.val[2] - red, color.val[1] - green, color.val[0] - blue
        uint32_t rgb = (color.val[2] << 16 | color.val[1] << 8 | color.val[0]);
        point.color = *reinterpret_cast<float*>(&rgb);
        // (x, y, z, r, g, b)
        pcl::PointXYZRGB pclPoint(point.point.x, point.point.y, point.point.z, color.val[2], color.val[1], color.val[0]);

        // average color of all points in the point cloud - (blue green red)
        _averageColor += (cv::Scalar(color.val[0], color.val[1], color.val[2]) - _averageColor) / (int)(_pointCloud.size() + 1);

        _pointCloud.push_back(point);
        _previous3dPoints.push_back(point);
        _pclPointCloudPtr->push_back(pclPoint);
    }
}

// obtain matches between previous 3d points and current image 2d points
void structureFromMotion::obtainMatches(imageFeatures features, cv::Mat descriptors3d, std::vector<cv::Point2d>& imagePoints2d, std::vector<cv::Point3d>& objectPoints3d, std::vector<pointInCloud> previous3dPoints)
{
    std::vector<std::vector<cv::DMatch>> matches;
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);

    matcher->knnMatch(descriptors3d, features.descriptors, matches, 2);

    for (std::vector<cv::DMatch>& matche : matches)
    {
        if (matche[0].distance < RATIO_THRESH * matche[1].distance)
        {
            imagePoints2d.push_back(features.keyPoints[matche[0].trainIdx].pt);
            objectPoints3d.push_back(previous3dPoints[matche[0].queryIdx].point);
        }
    }
}

// saves the Point cloud to file
void structureFromMotion::savePointCloud(std::string filePath)
{
    pcl::io::savePLYFileBinary(filePath, *_pclPointCloudPtr);
}
