#pragma once

#include "cameraPosition.h"

#include "ceres/ceres.h"
#include "ceres/rotation.h"


struct ReprojectionError {
    ReprojectionError(const cv::Mat& projection, const cv::Point2f& imagePoint)
        : _imagePoint(imagePoint), _projection(projection) {}

    template<typename T>
    bool operator()(const T* const point3d, T* residuals) const
 {
        T p[3];
        p[0] = _projection.at<double>(0, 0) * point3d[0] +
            _projection.at<double>(0, 1) * point3d[1] +
            _projection.at<double>(0, 2) * point3d[2] +
            _projection.at<double>(0, 3);
        p[1] = _projection.at<double>(1, 0) * point3d[0] +
            _projection.at<double>(1, 1) * point3d[1] +
            _projection.at<double>(1, 2) * point3d[2] +
            _projection.at<double>(1, 3);
        p[2] = _projection.at<double>(2, 0) * point3d[0] +
            _projection.at<double>(2, 1) * point3d[1] +
            _projection.at<double>(2, 2) * point3d[2] +
            _projection.at<double>(2, 3);

        residuals[0] = p[0] / p[2] - T(_imagePoint.x);
        residuals[1] = p[1] / p[2] - T(_imagePoint.y);
        return true;
    }

    static ceres::CostFunction* create(const cv::Mat& projection, const cv::Point2f& imagePoint)
    {
        return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3>(new ReprojectionError(projection, imagePoint)));
    }

    const cv::Point2f& _imagePoint;
    const cv::Mat& _projection;
};

class bundleAdjustment
{
private:

public:
    bundleAdjustment(std::vector<pointInCloud>& pointCloud, pcl::PointCloud<pcl::PointXYZRGB>& pclPointCloud, const std::vector<imageFeatures>& features);
    ~bundleAdjustment() = default;
};

