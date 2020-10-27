#include "cameraCalibration.h"


// extract corner points from chessboard images
int cameraCalibration::addChessboardPoints(const std::vector<std::string>& chessboardImages, cv::Size& boardSize)
{
    std::vector<cv::Point2f> imageCorners;
    std::vector<cv::Point3f> objectCorners;

    // chessboard corners are at (i,j,0)
    for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
            objectCorners.push_back(cv::Point3f(i, j, 0.0f));

    cv::Mat image; 
    int goodImages = 0;

    for (int i = 0; i < chessboardImages.size(); i++) // for all chessboards
    {
        image = cv::imread(chessboardImages[i], 0);

        bool found = cv::findChessboardCorners(image, boardSize, imageCorners);

        // refine the imageCorners to subpixel accuracy
        cv::cornerSubPix(image, imageCorners, cv::Size(5, 5), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
                                          30,    // max number of iterations
                                          0.1)); // min accuracy

        if (imageCorners.size() == boardSize.area()) // if the chessboard image is valid
        {
            // add chessboard image and scene points
            // 2D points
            imagePoints.push_back(imageCorners);
            // corresponding 3D scene points
            objectPoints.push_back(objectCorners);
            goodImages++;
        }
        ////Draw the corners
        ////cv::drawChessboardCorners(image, boardSize, imageCorners, found);
        ////cv::imshow("Corners on Chessboard", image);
        ////cv::waitKey(100);
    }
    imageSize = image.size();
    return goodImages;
}

double cameraCalibration::calibrate()
{
    // calibrate the camera
    return cv::calibrateCamera(
        objectPoints,
        imagePoints,
        imageSize,
        cameraMatrix,
        distortionCoefficients,
        cv::noArray(),
        cv::noArray(),
        cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT
    );
}

cv::Mat cameraCalibration::remap(const cv::Mat& image)
{
    cv::Mat undistorted;
    if(map1.empty() || map2.empty())
        cv::initUndistortRectifyMap(cameraMatrix, // computed camera matrix
                                    distortionCoefficients,   // computed distortion matrix
                                    cv::Mat(),    // optional rectification (none)
                                    cv::Mat(),    // camera matrix to generate undistorted
                                    image.size(),  // size of undistorted
                                    CV_32FC1,    // type of output map
                                    map1, map2); // the x and y mapping functions

    cv::remap(image, undistorted, map1, map2, cv::INTER_LINEAR);
    return undistorted;
}

// save calibration data
void cameraCalibration::save(std::string filePath)
{
    cv::FileStorage storage(filePath, cv::FileStorage::WRITE);
    storage << "cameraMatrix" << cameraMatrix;
    storage << "distortionCoefficients" << distortionCoefficients;
    storage.release();
}

// load calibration data
void cameraCalibration::load(std::string filePath)
{
    cv::FileStorage fs(filePath, cv::FileStorage::READ);
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distortionCoefficients"] >> distortionCoefficients;
    fs.release();
}
