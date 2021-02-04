#pragma once
#include "structureFromMotion.h"

#include "ceres/ceres.h"
#include "ceres/rotation.h"


struct ReprojectionError
{
    ReprojectionError(const cv::Point2f& imagePoint) : _imagePoint(imagePoint) {}

    template <typename T>
    bool operator()(const T* const camera, const T* const point3d, const T* const K, T* residuals) const
    {
        // camera[0,1,2] are the angle-axis rotation.
        T p[3];
        ceres::AngleAxisRotatePoint(camera, point3d, p);
        // camera[3,4,5] are the translation.
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];
        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        T xp = p[0] / p[2];
        T yp = p[1] / p[2];
        // Apply second and fourth order radial distortion.
        const T& fx = K[0];
        const T& fy = K[1];
        const T& cx = K[2];
        const T& cy = K[3];

        // Compute final projected point position.

        T predicted_x = fx * xp + cx;
        T predicted_y = fy * yp + cy;

        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - T(_imagePoint.x);
        residuals[1] = predicted_y - T(_imagePoint.y);
        return true;
    }

    static ceres::CostFunction* create(const cv::Point2f& imagePoint)
    {
        return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6, 3, 4>(new ReprojectionError(imagePoint)));
    }

    const cv::Point2f& _imagePoint;
};

class bundleAdjustment
{
private:
    void solveProblem();
    ceres::Problem _problem;
    double* _points;
    double* _intrinsics;
    double* _cameraPose;
public:
    bundleAdjustment(std::vector<pointInCloud>& pointCloud, pcl::PointCloud<pcl::PointXYZRGB>& pclPointCloud, std::vector<imageFeatures>& features, cameraCalibration& claib);
    ~bundleAdjustment();
};

