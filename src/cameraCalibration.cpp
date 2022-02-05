#include "cameraCalibration.h"
#include "features.h"

cv::Mat cameraCalibration::_cameraMatrix;
cv::Mat cameraCalibration::_distortionCoefficients;

// extract corner points from chessboard images - helps to get parameters about the camera that took the images 
int cameraCalibration::addChessboardPoints(const std::vector<std::string>& chessboardImages, cv::Size boardSize, const bool showMatchingFeature)
{
	std::vector<cv::Point2f> imageCorners;
	std::vector<cv::Point3f> objectCorners;

	// chessboard corners are at (i,j,0)
	for (int i = 0; i < boardSize.height; i++)
		for (int j = 0; j < boardSize.width; j++)
			objectCorners.push_back(cv::Point3f(i, j, 0.0f));

	cv::Mat image;
	int goodImages = 0;
	int screenWidth = getScreenWidth();

	for (int i = 0; i < chessboardImages.size(); i++) // for all chessboards - find the corners 
	{
		image = cv::imread(chessboardImages[i], cv::IMREAD_GRAYSCALE);
		if (!cv::findChessboardCorners(image, boardSize, imageCorners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS))
		{
			std::cout << "find chessboard corners faild" << std::endl;
			continue;
		}
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
		std::cout << "Chessboard image " << i + 1 << " / " << chessboardImages.size() << std::endl;

		if (showMatchingFeature) // Draw the corners
		{
			cv::drawChessboardCorners(image, boardSize, imageCorners, true);
			resizeWithAspectRatio(image, screenWidth);
			cv::imshow("Corners on chessboard", image);
			cv::waitKey(100);
		}
	}
	imageSize = image.size();
	cv::destroyAllWindows();
	return goodImages;
}

//calibrating usuing details we got from the load/chess board analyze
double cameraCalibration::calibrate()
{
	// calibrate the camera
	return cv::calibrateCamera(
		objectPoints,
		imagePoints,
		imageSize,
		_cameraMatrix,
		_distortionCoefficients,
		cv::noArray(),
		cv::noArray(),
		cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT
	);
}

//to get undistorted image ------ NOTE: Check**
cv::Mat cameraCalibration::remap(const cv::Mat& image)
{
	cv::Mat undistorted;
	if (map1.empty() || map2.empty())
		cv::initUndistortRectifyMap(_cameraMatrix, // computed camera matrix
			_distortionCoefficients,   // computed distortion matrix
			cv::Mat(),    // optional rectification (none)
			cv::Mat(),    // camera matrix to generate undistorted
			image.size(),  // size of undistorted
			CV_32FC1,    // type of output map
			map1, map2); // the x and y mapping functions

	cv::remap(image, undistorted, map1, map2, cv::INTER_LINEAR);
	return undistorted;
}

// save calibration data we analyzed to given file path
void cameraCalibration::save(std::string filePath)
{
	cv::FileStorage storage(filePath, cv::FileStorage::WRITE);
	storage << "cameraMatrix" << _cameraMatrix;
	storage << "distortionCoefficients" << _distortionCoefficients;
	storage.release();
}

// load calibration data from the given file path
void cameraCalibration::load(std::string filePath)
{
	cv::FileStorage fs(filePath, cv::FileStorage::READ);
	fs["cameraMatrix"] >> _cameraMatrix;
	fs["distortionCoefficients"] >> _distortionCoefficients;
	fs.release();
}

//get the fical from the camera metrix data
double cameraCalibration::getFocal()
{
	return _cameraMatrix.at<double>(0, 0);
}

//get specific PARAM from the camera metrix - NOTE: Check*
cv::Point2d cameraCalibration::getPrinciplePoint()
{
	return cv::Point2d(_cameraMatrix.at<double>(0, 2), _cameraMatrix.at<double>(1, 2));
}

//get camera matrix
cv::Mat& cameraCalibration::getCameraMatrix()
{
	/*| fx  0   cx |
	  | 0   fy  cy |
	  | 0   0   1  |*/
	return _cameraMatrix;
}

//get distortion coefficients
cv::Mat& cameraCalibration::getDistortionCoefficients()
{
	return _distortionCoefficients;
}

cv::Mat& cameraCalibration::estimateCameraMatrix(double focalLength, cv::Size imageSize)
{
	double cx = imageSize.width / 2;
	double cy = imageSize.height / 2;

	cv::Point2d pp(cx, cy);

	_cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

	_cameraMatrix.at<double>(0, 0) = focalLength;
	_cameraMatrix.at<double>(1, 1) = focalLength;
	_cameraMatrix.at<double>(0, 2) = cx;
	_cameraMatrix.at<double>(1, 2) = cy;

	return _cameraMatrix;
}
void cameraCalibration::downScale(double downScale)
{
	_cameraMatrix.at<double>(0, 0) /= downScale; //fx
	_cameraMatrix.at<double>(1, 1) /= downScale; //fy
	_cameraMatrix.at<double>(0, 2) /= downScale; //cx
	_cameraMatrix.at<double>(1, 2) /= downScale; //cy
}

cameraCalibration::~cameraCalibration()
{
	_cameraMatrix.release();
	_distortionCoefficients.release();
	map1.release();
	map2.release();
	for (int i = 0; i < objectPoints.size(); i++)
	{
		objectPoints[i].clear();
	}
	objectPoints.clear();
	for (int i = 0; i < imagePoints.size(); i++)
	{
		imagePoints[i].clear();
	}
	imagePoints.clear();
}

std::ostream& operator<<(std::ostream& out, const cameraCalibration& calib)
{
	out << "Calibration parms:" << std::endl;
	out << "Camera matrix:" << std::endl;
	out << calib.getCameraMatrix() << std::endl;
	out << "Distortion coefficients:" << std::endl;
	out << calib.getDistortionCoefficients() << std::endl;
	return out;
}
