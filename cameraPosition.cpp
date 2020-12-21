#include "cameraPosition.h"

//NOTE: COMMENT BOTH PROGRAMMERS!
void obtainMatches(std::vector<cv::KeyPoint>& kp1, std::vector<cv::KeyPoint>& kp2, cv::Mat& descriptors1, cv::Mat& descriptors2, std::vector<cv::Point2d>& points1, std::vector<cv::Point2d>& points2, std::vector<int>& points1_idx, std::vector<int>& points2_idx);

//builds pcl and reconstract camera position
//gets camera calib and vector of features(also matched)
cameraPosition::cameraPosition(cameraCalibration calib, std::vector<imageFeatures> features)
{
    matchingKeyPoints matchingKeyPoints = features[0].matchingKeyPoints;

    cv::Mat E, rotation, translation, mask;
	//getting essential metrix and recovers cam position
    E = findEssentialMat(matchingKeyPoints.currentKeyPoints, matchingKeyPoints.otherKeyPoints, calib.getFocal(), calib.getPP(), cv::RANSAC, 0.9, 3.0, mask);
    cv::recoverPose(E, matchingKeyPoints.currentKeyPoints, matchingKeyPoints.otherKeyPoints, rotation, translation, calib.getFocal(), calib.getPP(), mask);

    cv::Mat zeros = (cv::Mat_<double>(3, 1) << 0, 0, 0);

	//creates vector of cloud points
    std::vector<CloudPoint> all_3d_points;
	//create PCL which contains points - with XYZ(3d points) and each point also has Red Green Blue params(RGB)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    cv::Mat tran, tran1, points4D;

    hconcat(rotation, translation, tran);
    hconcat(calib.getCameraMatrix(), zeros, tran1);
    features[0].projection = tran1;
    features[1].projection = calib.getCameraMatrix() * tran;

    triangulatePoints(features[0].projection, features[1].projection, matchingKeyPoints.currentKeyPoints, matchingKeyPoints.otherKeyPoints, points4D);

    for (int i = 0; i < points4D.cols; i++) 
    {
        CloudPoint point;

        // convert point from homogeneous to 3d cartesian coordinates
        point.point = cv::Point3d(points4D.at<float>(0, i) / points4D.at<float>(3, i), points4D.at<float>(1, i) / points4D.at<float>(3, i), points4D.at<float>(2, i) / points4D.at<float>(3, i));

        std::vector<int> indices(features.size(), 0);
        indices[0] = matchingKeyPoints.currentKeyPointsIdx[i];
        indices[1] = matchingKeyPoints.otherKeyPointsIdx[i];
        point.index_of_2d_origin = indices;

        cv::Vec3b color = features[0].image.at<cv::Vec3b>(cv::Point(matchingKeyPoints.currentKeyPoints[i].x, matchingKeyPoints.currentKeyPoints[i].y));
       
        // color.val[2] is red, color.val[1] is green and color.val[0] is blue
        uint32_t rgb = (color.val[2] << 16 | color.val[1] << 8 | color.val[0]);
        point.color = *reinterpret_cast<float*>(&rgb);

        pcl::PointXYZRGB point_pcl(point.point.x, point.point.y, point.point.z, color.val[2], color.val[1], color.val[0]);

        all_3d_points.push_back(point);
        point_cloud_ptr->push_back(point_pcl);
    }


    std::vector<CloudPoint> previous_3d_points = all_3d_points;

 
    for (unsigned int i = 2; i < features.size() - 1; i++)
    {

        std::vector<cv::KeyPoint> keypoint3d;
        cv::KeyPoint key3d;
        cv::Mat descriptors3d;
        int posicion;

        for (unsigned int j = 0; j < previous_3d_points.size(); j++) 
        {
            posicion = previous_3d_points[j].index_of_2d_origin[i - 1];
            descriptors3d.push_back(features[i - 1].descriptors.row(posicion));
            key3d = features[i - 1].keyPoints[posicion];
            keypoint3d.push_back(key3d);
        }
        std::cout << "3d " << keypoint3d.size() << ", " << "kp " << features[i].keyPoints.size() << std::endl;


        std::vector<cv::Point2d> obj_new, scene_new;
        std::vector<int> obj_idx_new, scene_idx_new;
        obtainMatches(keypoint3d, features[i].keyPoints, descriptors3d, features[i].descriptors, obj_new, scene_new, obj_idx_new, scene_idx_new);

        cv::Mat rvec, tvec;
        std::vector<cv::Point3d> pnpPointcloud_valid;

        for (unsigned int j = 0; j < obj_idx_new.size(); j++) 
        {
            pnpPointcloud_valid.push_back(previous_3d_points[obj_idx_new[j]].point);
        }
        std::cout << pnpPointcloud_valid.size() << ", " << scene_new.size() << std::endl;
        cv::solvePnPRansac(pnpPointcloud_valid, scene_new, calib.getCameraMatrix(), calib.getDistortionCoefficients(), rvec, tvec, false, 300, 3.0); 
        //TO DO FIX - 19 error OpenCV(4.4.0-dev) Error: Unspecified error (> DLT algorithm needs at least 6 points for pose estimation from 3D-2D point correspondences. (expected: 'count >= 6'), where 'count' is 5 must be greater than or equal to '6' is 6) in void __cdecl cvFindExtrinsicCameraParams2(const struct CvMat*, const struct CvMat*, const struct CvMat*, const struct CvMat*, struct CvMat*, struct CvMat*, int), file C : \cv\opencv - master\modules\calib3d\src\calibration.cpp, line 1173

        std::cout << "Ransac done!" << endl;

        // Convert from Rodrigues format to rotation matrix and build the 3x4 Transformation matrix
        cv::Mat r_mat;
        cv::Rodrigues(rvec, r_mat);
        hconcat(r_mat, tvec, tran);

        // triangulate the points
        features[i].projection = calib.getCameraMatrix() * tran;

        cv::Mat newPoints4D;
        cout << "Image " << i - 1 << " with image " << i << ": " << features[i - 1].matchingKeyPoints.currentKeyPoints.size() << endl;
        triangulatePoints(features[i - 1].projection, features[i].projection, features[i - 1].matchingKeyPoints.currentKeyPoints, features[i - 1].matchingKeyPoints.otherKeyPoints, newPoints4D);

        previous_3d_points.clear();

        for (int j = 0; j < newPoints4D.cols; j++) {

            if ((features[i - 1].matchingKeyPoints.currentKeyPointsIdx[j] < features[i - 1].keyPoints.size())
                && (features[i - 1].matchingKeyPoints.currentKeyPointsIdx[j] >= 0)
                && (features[i - 1].matchingKeyPoints.otherKeyPointsIdx[j] < features[i].keyPoints.size())
                && (features[i - 1].matchingKeyPoints.otherKeyPointsIdx[j] >= 0)) {

                CloudPoint point;

                // convert point from homogeneous to 3d cartesian coordinates
                point.point = cv::Point3d(newPoints4D.at<float>(0, j) / newPoints4D.at<float>(3, j), newPoints4D.at<float>(1, j) / newPoints4D.at<float>(3, j), newPoints4D.at<float>(2, j) / newPoints4D.at<float>(3, j));

                // You only have a couple of images so the vector has to correspond only to these two images
                std::vector<int> indices(features.size(), 0);

                indices[i - 1] = features[i - 1].matchingKeyPoints.currentKeyPointsIdx[j];
                indices[i] = features[i - 1].matchingKeyPoints.otherKeyPointsIdx[j];
                point.index_of_2d_origin = indices;

                cv::Vec3b color = features[i - 1].image.at<cv::Vec3b>(cv::Point(features[i - 1].matchingKeyPoints.currentKeyPoints[i].x, features[i - 1].matchingKeyPoints.currentKeyPoints[i].y)); //???????

                // color.val[2] - red, color.val[1] - green, color.val[0] - blue
                uint32_t rgb = (color.val[2] << 16 | color.val[1] << 8 | color.val[0]);
                point.color = *reinterpret_cast<float*>(&rgb);
                // (x, y, z, r, g, b)
                pcl::PointXYZRGB point_pcl(point.point.x, point.point.y, point.point.z, color.val[2], color.val[1], color.val[0]);

                all_3d_points.push_back(point);
                previous_3d_points.push_back(point);
                point_cloud_ptr->push_back(point_pcl);

            }
        }
    }

    //display the point cloud
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(235, 235, 235);
    viewer->addPointCloud<pcl::PointXYZRGB>(point_cloud_ptr, "point cloud");
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