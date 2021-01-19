#include "bundleAdjustment.h"



bundleAdjustment::bundleAdjustment(std::vector<pointInCloud>& pointCloud, pcl::PointCloud<pcl::PointXYZRGB>& pclPointCloud, const std::vector<imageFeatures>& features, cameraCalibration& claib)
{
    double* cameraParams = new double[6 * features.size()];
    for (int i = 0; i < features.size(); i++)
    {
        cv::Mat rotationVector;
        cv::Rodrigues(features[i].rotation, rotationVector);
        cameraParams[6 * i + 0] = rotationVector.at<double>(0, 0);
        cameraParams[6 * i + 1] = rotationVector.at<double>(1, 0);
        cameraParams[6 * i + 2] = rotationVector.at<double>(2, 0);
        cameraParams[6 * i + 3] = features[i].translation.at<double>(0, 0);
        cameraParams[6 * i + 4] = features[i].translation.at<double>(1, 0);
        cameraParams[6 * i + 5] = features[i].translation.at<double>(2, 0);
    }

    _points = new double[3 * pointCloud.size()];
    for (int i = 0; i < pointCloud.size(); i++)
    {
        _points[3 * i + 0] = pointCloud[i].point.x;
        _points[3 * i + 1] = pointCloud[i].point.y;
        _points[3 * i + 2] = pointCloud[i].point.z;
    }

    double* KParams = new double[4];
    KParams[0] = claib.getCameraMatrix().at<double>(0,0); //fx
    KParams[1] = claib.getCameraMatrix().at<double>(1, 1); //fy
    KParams[2] = claib.getCameraMatrix().at<double>(0, 2); //cx
    KParams[3] = claib.getCameraMatrix().at<double>(1, 2); //cy

    ////// TO DELETE - it does not do anything //////
    // load extrinsics (rotations and translation)
    for (size_t i = 0; i < features.size(); ++i)
    {
        _problem.AddParameterBlock(&cameraParams[i*6], 6);
    }
    // fix the first camera.
    _problem.SetParameterBlockConstant(&cameraParams[0]);

    // load intrinsic
    _problem.AddParameterBlock(KParams, 4); // fx, fy, cx, cy
    ////// TO DELETE //////

    ceres::LossFunction* lossFunction = new ceres::HuberLoss(4);
    for (int i = 0; i < pointCloud.size(); i++)
    {
        ceres::CostFunction* costFunction = ReprojectionError::create(features[pointCloud[i].imageIndex].keyPoints[pointCloud[i].otherKeyPointsIdx].pt);
        _problem.AddResidualBlock(costFunction, lossFunction, &cameraParams[6 * pointCloud[i].imageIndex], &_points[3 * i], KParams);
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
        // To DO
        //cv::Mat rotationVector = (cv::Mat_<double>(3, 1) << cameraParams[6 * i + 0], cameraParams[6 * i + 1], cameraParams[6 * i + 3]);
        //cv::Rodrigues(features[i].rotation, rotationVector);
        //cameraParams[6 * i + 0] = rotationVector.at<double>(0, 0);
        //cameraParams[6 * i + 1] = rotationVector.at<double>(1, 0);
        //cameraParams[6 * i + 2] = rotationVector.at<double>(2, 0);

        // copy back the translation
        cv::Mat translation = (cv::Mat_<double>(3, 1) << cameraParams[6 * i + 3], cameraParams[6 * i + 4], cameraParams[6 * i + 5]);
        translation.copyTo(features[i].translation);
    }
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
}
