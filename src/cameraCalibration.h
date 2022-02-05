#pragma once

#include <iostream>
#include <vector>
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/core.hpp>
#include "opencv2/highgui.hpp"

class cameraCalibration
{
private:
	std::vector<std::vector<cv::Point3f>> objectPoints; // store x,y,z points
	std::vector<std::vector<cv::Point2f>> imagePoints; // store x,y points

	static cv::Mat _cameraMatrix;
	static cv::Mat _distortionCoefficients;

	cv::Mat map1, map2;
	cv::Size imageSize;
public:
	cameraCalibration() = default;
	~cameraCalibration();
	int addChessboardPoints(const std::vector<std::string>& filelist, cv::Size boardSize, const bool showMatchingFeature = false);
	double calibrate();
	cv::Mat remap(const cv::Mat& image);
	static cv::Mat& getCameraMatrix();
	static cv::Mat& getDistortionCoefficients();
	static cv::Point2d getPrinciplePoint();
	double getFocal();
	void save(std::string filePath);
	void load(std::string filePath);
	cv::Mat& estimateCameraMatrix(double focalLength, cv::Size imageSize);
	void downScale(double downScale = 2);
	friend std::ostream& operator<<(std::ostream& out, const cameraCalibration& calib);
};