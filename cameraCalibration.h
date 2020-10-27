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
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;

    cv::Mat cameraMatrix;
    cv::Mat distortionCoefficients;

    cv::Mat map1, map2;
    cv::Size imageSize;
  public:

      cameraCalibration() = default;
      int addChessboardPoints(const std::vector<std::string>& filelist, cv::Size& boardSize);
      double calibrate();
      cv::Mat remap(const cv::Mat& image);
      cv::Mat getCameraMatrix() { return cameraMatrix; }
      cv::Mat getDistortionCoefficients() { return distortionCoefficients; }
      void save(std::string filePath);
      void load(std::string filePath);
};
