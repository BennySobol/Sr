
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

std::string objectImagesPath = "C:\\Users\\BennySobol\\Desktop\\dataset\\‏fountain-P5\\images\\";
bool isSorted = true;

int main(int argc, char* argv[])
{
    try 
    {
        cameraCalibration cameraCalibrator;
        if (fs::exists(xmlFilePath)) // load calibration
        {
            cameraCalibrator.load(xmlFilePath);
        }

        else
        { // calibrate camera
            std::vector<std::string> ImagesPath = loadImages::getInstance()->load(chessboardImagesPath, true);
            cameraCalibrator.addChessboardPoints(ImagesPath, cv::Size(9, 6)); // size of inner chessboard corners

            std::cout << "Error: " << cameraCalibrator.calibrate() << "\n";
            cameraCalibrator.save("calib.xml");
        }
       
        std::cout << "Calibration parms:\n CameraMatrix:\n" << cameraCalibrator.getCameraMatrix() << "\nDistortionCoefficients\n" << cameraCalibrator.getDistortionCoefficients() << "\n";

        auto images = loadImages::getInstance()->load(objectImagesPath, isSorted);
        std::cout << "Images were loaded\n";
        features features(images);
        std::cout << "Features were extracted\n";
        features.matchFeatures(cameraCalibrator);
        std::cout << "Images were sorted\n";
        auto imagesFeatures = features.getFeatures();

        cameraPosition cameraPosition(cameraCalibrator, imagesFeatures);
    }
    catch (const std::exception& e) 
    {
        std::cout << "ERROR: " << e.what() << '\n';
    }
    return 0;
}