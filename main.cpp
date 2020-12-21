#include "loadImages.h"
#include "features.h"
#include "cameraCalibration.h"
#include "cameraPosition.h"

std::string xmlFilePath = ""; //camera calbration file location
std::string chessboardImagesPath = ""; // chess board images for camera calibration
std::string objectImagesPath = "C:\\Users\\BennySobol\\Desktop\\dataset\\fountain-P5\\images"; // Images of object surface to extract points cloud from
bool isSorted = true;
double focalLength = 2759.48;

int main(int argc, char* argv[])
{
    try
    {
        cameraCalibration cameraCalibrator;

        if (fs::exists(xmlFilePath)) // load calibration - if exsits
        {
            cameraCalibrator.load(xmlFilePath);
        }
        else if(chessboardImagesPath != "")
        { // calibrate camera
            std::vector<std::string> ImagesPath = loadImages::getInstance()->load(chessboardImagesPath, true);
            cameraCalibrator.addChessboardPoints(ImagesPath, cv::Size(9, 6)); // size of inner chessboard corners
			//print if there are errors, and save calibration file
            std::cout << "Error: " << cameraCalibrator.calibrate() << "\n";
            cameraCalibrator.save("calib.xml");
        }
       
		////print camera calibration params and camera metrix extracted from calibration
  //      std::cout << "Calibration parms:\n CameraMatrix:\n" << cameraCalibrator.getCameraMatrix() << "\nDistortionCoefficients\n" << cameraCalibrator.getDistortionCoefficients() << "\n";

		//load images to images
        auto images = loadImages::getInstance()->load(objectImagesPath, isSorted);
        std::cout << "Images were loaded\n";
		//extract features from every image
        features features(images);
        std::cout << "Features were extracted\n";

        // cameraMatrix from focal length
        cameraCalibrator.estimateCameraMatrix(focalLength, features.getFeatures()[0].image.size());

		//find matching features between every 2 images - do it to all images
        features.matchFeatures(cameraCalibrator);
        std::cout << "Images were sorted\n";
		//load the features
        auto imagesFeatures = features.getFeatures();
		//get the PCL and display it using camera position reconstract
        cameraPosition cameraPosition(cameraCalibrator, imagesFeatures);
    }
    catch (const std::exception& e)
    {
        std::cout << "ERROR: " << e.what() << '\n';
    }
    return 0;
}