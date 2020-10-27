
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


#include "loadImages.h"
#include "features.h"
#include "cameraCalibration.h"
#include "cameraPosition.h"

std::string xmlFilePath = "calib.xml";
std::string chessboardImagesPath = "C:\\chessboardImages";
std::string objectImagesPath = "C:\\objectImages";

int main(int argc, char* argv[])
{
    cameraCalibration cameraCalibrator;
    if (fs::exists(xmlFilePath)) // load calibration
    {
        cameraCalibrator.load(xmlFilePath);
    }

    else
    { // calibrate camera
        std::vector<std::string> ImagesPath = loadImages::getInstance()->load(chessboardImagesPath);
        cameraCalibrator.addChessboardPoints(ImagesPath, cv::Size(9, 6)); // size of inner chessboard corners

        std::cout << "Error: " << cameraCalibrator.calibrate() << "\n";
        cameraCalibrator.save("calib.xml");
    }

    std::cout << "Calibration parms:\n CameraMatrix:\n" << cameraCalibrator.getCameraMatrix() << "\nDistortionCoefficients\n" << cameraCalibrator.getDistortionCoefficients() << "\n";


    auto images = loadImages::getInstance()->load(objectImagesPath);
    std::cout << "Images were loaded\n";
    features features(images);
    std::cout << "Features were extracted\n";
    features.matchFeatures();
    std::cout << "Images were sorted\n";
    auto imagesFeatures = features.getFeatures();

    cameraPosition cameraPosition(cameraCalibrator.getCameraMatrix(), imagesFeatures);
    return 0;
}