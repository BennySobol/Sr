#pragma once

#include "cameraPosition.h"

#include "ceres/ceres.h"
#include "ceres/rotation.h"


struct ReprojectionError {
    ReprojectionError(const cv::Mat& cam_info, const cv::Point2f& img_point)
        : camera_info(cam_info), image_point(img_point), proj(cam_info) {}

    template<typename T>
    bool operator()(const T* const point3d, T* residuals) const {
        T p[3];
        p[0] = proj.at<double>(0, 0) * point3d[0] +
            proj.at<double>(0, 1) * point3d[1] +
            proj.at<double>(0, 2) * point3d[2] +
            proj.at<double>(0, 3);
        p[1] = proj.at<double>(1, 0) * point3d[0] +
            proj.at<double>(1, 1) * point3d[1] +
            proj.at<double>(1, 2) * point3d[2] +
            proj.at<double>(1, 3);
        p[2] = proj.at<double>(2, 0) * point3d[0] +
            proj.at<double>(2, 1) * point3d[1] +
            proj.at<double>(2, 2) * point3d[2] +
            proj.at<double>(2, 3);

        residuals[0] = p[0] / p[2] - T(image_point.x);
        residuals[1] = p[1] / p[2] - T(image_point.y);
        return true;
    }

    static ceres::CostFunction* create(const cv::Mat& cam_info, const cv::Point2f& img_point)
    {
        return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3>(
            new ReprojectionError(cam_info, img_point)));
    }

    const cv::Mat& camera_info;
    const cv::Point2f& image_point;
    cv::Mat proj;
};

class bundleAdjustment
{
private:

public:
    bundleAdjustment(std::vector<pointInCloud>& pointCloud, pcl::PointCloud<pcl::PointXYZRGB>& pclPointCloud, const std::vector<cv::Mat>& cameras, const std::vector<imageFeatures>& features);
    ~bundleAdjustment() = default;
};

