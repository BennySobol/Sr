#include "features.h"
#include "cameraCalibration.h"
#include "structureFromMotion.h"
#include "surfaceReconstruction.h"
#include "loadImages.h"


std::string xmlFilePath = "../calib.xml"; //camera calbration file location
std::string chessboardImagesPath = ""; // chess board images for camera calibration
std::string objectImagesPath = "C:\\Users\\BennySobol\\Desktop\\dataset\\fountain-P5\\images"; // Images of object surface to extract points cloud from
double focalLength = 0;
bool isSorted = true;
bool optimization = false;
bool showMatchFeatures = false;

//TO DO
// main params => -b -s -h -s -o -cf=1234 -cxml=calib.xml -cchessboard=C:\\chessboard  -u=C:\\images 

int main(int argc, char* argv[])
{
    try
    {
        cameraCalibration cameraCalibrator;

		//load images
        std::vector<std::string> images = loadImages::getInstance()->load(objectImagesPath, isSorted);
        std::cout << "Images path were loaded\n";
		//extract features from every image
        features features(images);

        //features features("features.xml"); //TO DO - FIX
        //features.load("features.xml");

        std::cout << "Features were extracted\n";
        if (fs::exists(xmlFilePath)) // load calibration - if exsits
        {
            cameraCalibrator.load(xmlFilePath);
        }
        else if (chessboardImagesPath != "") // calibrate camera
        {
            std::vector<std::string> ImagesPath = loadImages::getInstance()->load(chessboardImagesPath, true);
            cameraCalibrator.addChessboardPoints(ImagesPath, cv::Size(9, 6)); // size of inner chessboard corners
            //print if there are errors, and save calibration file
            std::cout << "Error: " << cameraCalibrator.calibrate() << "\n";
            cameraCalibrator.save("calib.xml");
        }
        else if (focalLength != 0) // cameraMatrix from focal length
        {
            cameraCalibrator.estimateCameraMatrix(focalLength, features.getFeatures()[0].image.size());
        }
        else
        {
            throw std::exception("There mast be calibration parms");
        }
        // print camera calibration params and camera metrix extracted from calibration
        std::cout << "Calibration parms:\n CameraMatrix:\n" << cameraCalibrator.getCameraMatrix() << "\nDistortionCoefficients\n" << cameraCalibrator.getDistortionCoefficients() << "\n";

		// find matching features between every 2 images - do it to all images
        features.matchFeatures(cameraCalibrator, optimization, showMatchFeatures);
        std::cout << "Images features were matched\n";

        // features.save("features.xml");

		// load the features
        std::vector <imageFeatures> imagesFeatures = features.getFeatures();
        std::cout << "Starting structure from motion\n";
		// get the PCL and display it using camera position reconstract
        structureFromMotion structureFromMotion(cameraCalibrator, imagesFeatures, optimization);
        structureFromMotion.savePointCloud(objectImagesPath + "\\pclSaved.ply");
        std::cout << "Structure from motion is finished\n";
        std::cout << "Starting surface reconstruction\n";
        // surface reconstruction
        surfaceReconstruction::surfaceReconstruction(objectImagesPath + "\\pclSaved.ply", objectImagesPath + "\\offSaved.off", 2);
        std::cout << "Surface reconstruction is finished\n";
    }
    catch (const std::exception& e)
    {
        std::cout << "ERROR: " << e.what() << '\n';
    }

    return 0;
}