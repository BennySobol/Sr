#include "cameraPosition.h"

//////////////////////////////////////////// TO DO ///////////////////////////////////////////////////
//  we shuold add the first pair of images 3d points to the point cloud and then scale the new 3d points to fit



int nextBestImage(int index, std::vector<imageFeatures> _features)
{
    int maxSize = 0;
    int maxSizeIndex = -1;
    for (int i = 0; i < _features[index].matchesKeyPoints.size(); i++)
    {
        int size = _features[index].matchesKeyPoints[i].currentKeyPoints.size();
        if (size > maxSize)
        {
            maxSize = size;
            maxSizeIndex = i;
        }
    }
    return maxSizeIndex;
}

cameraPosition::cameraPosition(cv::Mat K, std::vector<imageFeatures> _features)
{

    // Recover motion between first to second image
    cv::Mat Transformation = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat Projection = K * cv::Mat::eye(3, 4, CV_64F);


    auto& current = _features[0];
    int nextImage = nextBestImage(0, _features);
    auto& next = _features[0];

    std::vector<cv::Point2f>& point1 = current.matchesKeyPoints[nextImage].currentKeyPoints;
    std::vector<cv::Point2f>& point2 = current.matchesKeyPoints[nextImage].otherKeyPoints;


    // pose from next to current
    cv::Mat mask;
    cv::Mat E = findEssentialMat(point2, point1, K.at<double>(0, 0), cv::Point2d(K.at<double>(0, 2), K.at<double>(1, 2)), cv::RANSAC, 0.999, 1.0, mask);
    cv::Mat local_R, local_t;

    recoverPose(E, point2, point1, local_R, local_t, K.at<double>(0, 0), cv::Point2d(K.at<double>(0, 2), K.at<double>(1, 2)), mask);

    // local tansform
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    local_R.copyTo(T(cv::Range(0, 3), cv::Range(0, 3)));
    local_t.copyTo(T(cv::Range(0, 3), cv::Range(3, 4)));

    // accumulate transform
    cv::Mat nextTransformation = Transformation * T;


    // make projection matrix
    cv::Mat R = nextTransformation(cv::Range(0, 3), cv::Range(0, 3));
    cv::Mat t = nextTransformation(cv::Range(0, 3), cv::Range(3, 4));

    cv::Mat P(3, 4, CV_64F);

    P(cv::Range(0, 3), cv::Range(0, 3)) = R.t();
    P(cv::Range(0, 3), cv::Range(3, 4)) = -R.t() * t;
    P = K * P;

    cv::Mat nextProjection = P;

    cv::Mat points4D;
    triangulatePoints(Projection, nextProjection, point1, point2, points4D);

    // display point cloud

    // prepare a viewer
    pcl::visualization::PCLVisualizer viewer("Viewer");
    viewer.setBackgroundColor(255, 255, 255);

    // create point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->points.resize(points4D.rows);

    for (int i = 0; i < points4D.rows; i++) {
        pcl::PointXYZRGB& point = cloud->points[i];

        point.x = points4D.at<float>(0, i);
        point.y = points4D.at<float>(1, i);
        point.z = points4D.at<float>(2, i);
        point.r = 0;
        point.g = 0;
        point.b = 255;
    }

    viewer.addPointCloud(cloud, "Point Cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Point Cloud"); viewer.addCoordinateSystem(1.0);

    viewer.initCameraParameters();
    while (!viewer.wasStopped()) {
        viewer.spin();
    }
}


