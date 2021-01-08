#include "bundleAdjustment.h"



bundleAdjustment::bundleAdjustment(std::vector<pointInCloud>& pointCloud, pcl::PointCloud<pcl::PointXYZRGB>& pclPointCloud, const std::vector<cv::Mat>& cameras, const std::vector<imageFeatures>& features)
{
    ceres::Problem problem;

    double* points = new double[3 * pointCloud.size()];
    for (size_t i = 0; i < pointCloud.size(); ++i) {
        points[3 * i + 0] = pointCloud[i].point.x;
        points[3 * i + 1] = pointCloud[i].point.y;
        points[3 * i + 2] = pointCloud[i].point.z;
    }

    for (size_t i = 0; i < pointCloud.size(); ++i)
    {
        ceres::CostFunction* cost_function = ReprojectionError::create(
            cameras[pointCloud[i].i - 1],
            features[pointCloud[i].i - 1].matchingKeyPoints.currentKeyPoints[pointCloud[i].j]);
        problem.AddResidualBlock(cost_function,
            new ceres::CauchyLoss(0.5),
            &points[3 * i]);
    }

    // Make Ceres automatically detect the bundle structure. Note that the
    // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
    // for standard bundle adjustment problems.
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 500;
    options.eta = 1e-2;
    options.max_solver_time_in_seconds = 3500;
    options.logging_type = ceres::LoggingType::SILENT;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // std::cout << std::endl;
    std::cout << summary.BriefReport();
    // std::cout << summary.FullReport();
    std::cout << ", SolverTime = " << summary.total_time_in_seconds;


    if (summary.termination_type != ceres::CONVERGENCE) {
        std::cerr << "Bundle adjustment failed." << std::endl;
        std::cout << summary.FullReport();
        return;
    }

    // copy back the points
    for (size_t i = 0; i < pointCloud.size(); ++i) {
        pointCloud[i].point.x = points[3 * i];
        pointCloud[i].point.y = points[3 * i + 1];
        pointCloud[i].point.z = points[3 * i + 2];

        pclPointCloud[i].x = points[3 * i];
        pclPointCloud[i].y = points[3 * i + 1];
        pclPointCloud[i].z = points[3 * i + 2];
    }

    delete[] points;
}
