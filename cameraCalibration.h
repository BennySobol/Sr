#pragma once

#include <iostream>
#include <vector>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class cameraCalibration
{
private:
    std::vector<std::vector<cv::Point3f>> objectPoints; // store x,y,z points
    std::vector<std::vector<cv::Point2f>> imagePoints; // store x,y points

    cv::Mat _cameraMatrix;
    cv::Mat _distortionCoefficients;

    cv::Mat map1, map2;
    cv::Size imageSize;
  public:

      cameraCalibration() = default;
      int addChessboardPoints(const std::vector<std::string>& filelist, cv::Size& boardSize);
      double calibrate();
      cv::Mat remap(const cv::Mat& image);
      cv::Mat getCameraMatrix();
      cv::Mat getDistortionCoefficients();
      cv::Point2d cameraCalibration::getPP();
      float getFocal();
      void save(std::string filePath);
      void load(std::string filePath);
};
