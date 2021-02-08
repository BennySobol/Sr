#include "bundleAdjustment.h"


// inspiration from a tutorial at http://ceres-solver.org/nnls_tutorial.html#bundle-adjustment
bundleAdjustment::bundleAdjustment(std::vector<pointInCloud>& pointCloud, pcl::PointCloud<pcl::PointXYZRGB>& pclPointCloud, std::vector<imageFeatures>& features, cameraCalibration& claib)
{
    _cameraPose = new double[6 * features.size()];
    for (int i = 0; i < features.size(); i++)
    {
        cv::Mat rotationVector;
        cv::Rodrigues(features[i].rotation, rotationVector);
        _cameraPose[6 * i + 0] = rotationVector.at<double>(0, 0);
        _cameraPose[6 * i + 1] = rotationVector.at<double>(1, 0);
        _cameraPose[6 * i + 2] = rotationVector.at<double>(2, 0);
        _cameraPose[6 * i + 3] = features[i].translation.at<double>(0, 0);
        _cameraPose[6 * i + 4] = features[i].translation.at<double>(1, 0);
        _cameraPose[6 * i + 5] = features[i].translation.at<double>(2, 0);
    }

    _points = new double[3 * pointCloud.size()];
    for (int i = 0; i < pointCloud.size(); i++)
    {
        _points[3 * i + 0] = pointCloud[i].point.x;
        _points[3 * i + 1] = pointCloud[i].point.y;
        _points[3 * i + 2] = pointCloud[i].point.z;
    }

    _intrinsics = new double[4];
    _intrinsics[0] = claib.getCameraMatrix().at<double>(0,0); //fx
    _intrinsics[1] = claib.getCameraMatrix().at<double>(1, 1); //fy
    _intrinsics[2] = claib.getCameraMatrix().at<double>(0, 2); //cx
    _intrinsics[3] = claib.getCameraMatrix().at<double>(1, 2); //cy

    ////// TO DELETE - it does not do anything //////
    // load extrinsics (rotations and translation)
    for (size_t i = 0; i < features.size(); ++i)
    {
        _problem.AddParameterBlock(&_cameraPose[i*6], 6);
    }
    // fix the first camera.
    _problem.SetParameterBlockConstant(&_cameraPose[0]);

    // load intrinsic
    _problem.AddParameterBlock(_intrinsics, 4); // fx, fy, cx, cy
    ////// TO DELETE //////

    ceres::LossFunction* lossFunction = new ceres::HuberLoss(4);
    for (int i = 0; i < pointCloud.size(); i++)
    {
        ceres::CostFunction* costFunction = ReprojectionError::create(features[pointCloud[i].imageIndex].keyPoints[pointCloud[i].otherKeyPointsIdx].pt);
        _problem.AddResidualBlock(costFunction, lossFunction, &_cameraPose[6 * pointCloud[i].imageIndex], &_points[3 * i], _intrinsics);
    }

    solveProblem();
  
    // copy back the points
    for (int i = 0; i < pointCloud.size(); i++)
    {
        pointCloud[i].point.x = _points[3 * i];
        pointCloud[i].point.y = _points[3 * i + 1];
        pointCloud[i].point.z = _points[3 * i + 2];

        pclPointCloud[i].x = _points[3 * i];
        pclPointCloud[i].y = _points[3 * i + 1];
        pclPointCloud[i].z = _points[3 * i + 2];
    }

    for (int i = 0; i < features.size(); i++)
    {
        // copy back the rotation
        cv::Mat rotationVector = (cv::Mat_<double>(3, 1) << _cameraPose[6 * i + 0], _cameraPose[6 * i + 1], _cameraPose[6 * i + 2]);
        cv::Rodrigues(rotationVector, features[i].rotation);

        // copy back the translation
        cv::Mat translation = (cv::Mat_<double>(3, 1) << _cameraPose[6 * i + 3], _cameraPose[6 * i + 4], _cameraPose[6 * i + 5]);
        translation.copyTo(features[i].translation);
    }
    // copy back the intrinsics
    claib.getCameraMatrix().at<double>(0, 0) = _intrinsics[0]; //fx
    claib.getCameraMatrix().at<double>(1, 1) = _intrinsics[1]; //fy
    claib.getCameraMatrix().at<double>(0, 2) = _intrinsics[2]; //cx
    claib.getCameraMatrix().at<double>(1, 2) = _intrinsics[3]; //cy
}


void bundleAdjustment::solveProblem()
{
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 500;
    options.eta = 1e-2;
    options.max_solver_time_in_seconds = 3500;
    options.logging_type = ceres::LoggingType::SILENT;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &_problem, &summary);

    std::cout << summary.BriefReport() << ", SolverTime = " << summary.total_time_in_seconds << std::endl;

    if (summary.termination_type != ceres::CONVERGENCE)
    {
        std::cout << "Bundle adjustment failed." << std::endl;
        std::cout << summary.FullReport();
        return;
    }
}

bundleAdjustment::~bundleAdjustment()
{
    delete[] _points;
    delete[] _intrinsics;
    delete[] _cameraPose;
}
