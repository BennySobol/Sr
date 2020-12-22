#include "cameraPosition.h"


//builds pcl and reconstract camera position
//gets camera calib and vector of features(also matched)
cameraPosition::cameraPosition(cameraCalibration calib, std::vector<imageFeatures> features)
{
    cv::Mat E, rotation, translation, mask;
    //getting essential metrix and recovers cam position
    E = findEssentialMat(features[0].matchingKeyPoints.currentKeyPoints, features[0].matchingKeyPoints.otherKeyPoints, calib.getFocal(), calib.getPP(), cv::RANSAC, 0.9, 3.0, mask);
    cv::recoverPose(E, features[0].matchingKeyPoints.currentKeyPoints, features[0].matchingKeyPoints.otherKeyPoints, rotation, translation, calib.getFocal(), calib.getPP(), mask);

    cv::Mat zeros = (cv::Mat_<double>(3, 1) << 0, 0, 0);

    std::vector<CloudPoint> previous3dPoints;

    //initialize the pcl point cloud ptr which contains points - with XYZ(3d points) and each point also has Red Green Blue params(RGB)
    _pclPointCloudPtr.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    cv::Mat tran, tran1;
    cv::Mat points4D;

    hconcat(rotation, translation, tran);
    hconcat(calib.getCameraMatrix(), zeros, tran1);
    features[0].projection = tran1;
    features[1].projection = calib.getCameraMatrix() * tran;

    for (unsigned int i = 1; i < features.size() - 1; i++)
    {
        if (i > 1)
        {
            std::vector<cv::KeyPoint> keyPoint3d;
            cv::Mat descriptors3d;

            for (unsigned int j = 0; j < previous3dPoints.size(); j++)
            {
                int position = previous3dPoints[j].indexOf2dOrigin[i - 1];
                descriptors3d.push_back(features[i - 1].descriptors.row(position));
                keyPoint3d.push_back(features[i - 1].keyPoints[position]); // push the 3d key
            }
            std::cout << "3d " << keyPoint3d.size() << ", " << "kp " << features[i].keyPoints.size() << std::endl;

            std::vector<cv::Point2d> obj_new, scene_new;
            std::vector<int> obj_idx_new, scene_idx_new;
            obtainMatches(keyPoint3d, features[i].keyPoints, descriptors3d, features[i].descriptors, obj_new, scene_new, obj_idx_new, scene_idx_new);

            cv::Mat rvec, tvec;
            std::vector<cv::Point3d> pnpPointcloud_valid;

            for (unsigned int j = 0; j < obj_idx_new.size(); j++)
            {
                pnpPointcloud_valid.push_back(previous3dPoints[obj_idx_new[j]].point);
            }
            std::cout << pnpPointcloud_valid.size() << ", " << scene_new.size() << std::endl;
            cv::solvePnPRansac(pnpPointcloud_valid, scene_new, calib.getCameraMatrix(), calib.getDistortionCoefficients(), rvec, tvec, false, 300, 3.0);
            //TO DO FIX - 19 error OpenCV(4.4.0-dev) Error: Unspecified error (> DLT algorithm needs at least 6 points for pose estimation from 3D-2D point correspondences. (expected: 'count >= 6'), where 'count' is 5 must be greater than or equal to '6' is 6) in void __cdecl cvFindExtrinsicCameraParams2(const struct CvMat*, const struct CvMat*, const struct CvMat*, const struct CvMat*, struct CvMat*, struct CvMat*, int), file C : \cv\opencv - master\modules\calib3d\src\calibration.cpp, line 1173

            std::cout << "Ransac done!" << endl;

            // Convert from Rodrigues format to rotation matrix and build the 3x4 Transformation matrix
            cv::Rodrigues(rvec, rotation);
            hconcat(rotation, tvec, tran);

            features[i].projection = calib.getCameraMatrix() * tran;

            previous3dPoints.clear();
        }

        cout << "Image " << i - 1 << " with image " << i << ": " << features[i - 1].matchingKeyPoints.currentKeyPoints.size() << endl;

        // triangulate the points
        triangulatePoints(features[i - 1].projection, features[i].projection, features[i - 1].matchingKeyPoints.currentKeyPoints, features[i - 1].matchingKeyPoints.otherKeyPoints, points4D);

        for (int j = 0; j < points4D.cols; j++) {

            if ((features[i - 1].matchingKeyPoints.currentKeyPointsIdx[j] < features[i - 1].keyPoints.size())
                && (features[i - 1].matchingKeyPoints.currentKeyPointsIdx[j] >= 0)
                && (features[i - 1].matchingKeyPoints.otherKeyPointsIdx[j] < features[i].keyPoints.size())
                && (features[i - 1].matchingKeyPoints.otherKeyPointsIdx[j] >= 0)) {

                CloudPoint point;

                // convert point from homogeneous to 3d cartesian coordinates
                point.point = cv::Point3d(points4D.at<float>(0, j) / points4D.at<float>(3, j), points4D.at<float>(1, j) / points4D.at<float>(3, j), points4D.at<float>(2, j) / points4D.at<float>(3, j));

                // You only have a couple of images so the vector has to correspond only to these two images
                std::vector<int> indices(features.size(), 0);

                indices[i - 1] = features[i - 1].matchingKeyPoints.currentKeyPointsIdx[j];
                indices[i] = features[i - 1].matchingKeyPoints.otherKeyPointsIdx[j];
                point.indexOf2dOrigin = indices;

                cv::Vec3b color = features[i - 1].image.at<cv::Vec3b>(cv::Point(features[i - 1].matchingKeyPoints.currentKeyPoints[j].x, features[i - 1].matchingKeyPoints.currentKeyPoints[j].y));

                // color.val[2] - red, color.val[1] - green, color.val[0] - blue
                uint32_t rgb = (color.val[2] << 16 | color.val[1] << 8 | color.val[0]);
                point.color = *reinterpret_cast<float*>(&rgb);
                // (x, y, z, r, g, b)
                pcl::PointXYZRGB pclPoint(point.point.x, point.point.y, point.point.z, color.val[2], color.val[1], color.val[0]);

                _pointCloud.push_back(point);
                previous3dPoints.push_back(point);
                _pclPointCloudPtr->push_back(pclPoint);

            }
            else
            {
                std::cout << "this is a test of the if";
            }
        }
    }

    showPointCloud();
}


//display the point cloud
void cameraPosition::showPointCloud()
{
    //display the point cloud
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(235, 235, 235);
    viewer->addPointCloud<pcl::PointXYZRGB>(_pclPointCloudPtr, "point cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point cloud");
    viewer->initCameraParameters();
    viewer->setShowFPS(true);
    viewer->setWindowName("point cloud");

    while (!viewer->wasStopped()) {
        viewer->spin();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    cv::destroyAllWindows();
}

void obtainMatches(std::vector<cv::KeyPoint>& kp1, std::vector<cv::KeyPoint>& kp2, cv::Mat& descriptors1, cv::Mat& descriptors2, std::vector<cv::Point2d>& points1, std::vector<cv::Point2d>& points2, std::vector<int>& points1_idx, std::vector<int>& points2_idx) {

    cv::BFMatcher matcher(cv::NORM_L2, false);
    std::vector<std::vector< cv::DMatch > > matches;
    matcher.knnMatch(descriptors1, descriptors2, matches, 2);

    std::vector<cv::DMatch > Best_Matches;
    for (int k = 0; k < matches.size(); k++) {

        float dis1 = matches[k][0].distance;
        float dis2 = matches[k][1].distance;

        if (((dis1 < 300.0 && dis1 > 0) || (dis2 < 300.0 && dis2 > 0)) && (dis2 / dis1 > 1.5))
            Best_Matches.push_back(matches[k][0]);
    }
    for (int l = 0; l < Best_Matches.size(); l++) {
        points1.push_back(kp1[Best_Matches[l].queryIdx].pt);
        points1_idx.push_back(Best_Matches[l].queryIdx);

        points2.push_back(kp2[Best_Matches[l].trainIdx].pt);
        points2_idx.push_back(Best_Matches[l].trainIdx);
    }
}