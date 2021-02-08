#include "structureFromMotion.h"
#include "surfaceReconstruction.h"
#include "meshTexturing.h"
#include "loadImages.h"

// TO DO - put in setings file
double downScaleFactor = 3;
std::string chessboardImagesPath = ""; // chess board images for camera calibration
std::string objectImagesPath = "C:\\Users\\BennySobol\\Desktop\\dataset\\desk"; // Folder with images of an object
double focalLength = 4308;
bool isSorted = false;
bool showMatchFeatures = false;


int main(int argc, char* argv[])
{
    try
    {
        fs::create_directories(objectImagesPath + "\\output"); // create directories for output
        cameraCalibration cameraCalibrator;

		//load images
        std::vector<std::string> images = loadImages::getInstance()->load(objectImagesPath, isSorted);
        std::cout << "Images path were loaded\n";
		//extract features from every image
        features features(images, downScaleFactor);

        std::cout << "Features were extracted\n";
        if (fs::exists(objectImagesPath + "\\calib.xml")) // load calibration - if exsits
        {
            cameraCalibrator.load(objectImagesPath + "\\calib.xml");
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
        cameraCalibrator.downScale(downScaleFactor);

        // print camera params
        std::cout << "Calibration parms:" << std::endl;
        std::cout << "Camera matrix:" << std::endl;
        std::cout << cameraCalibrator.getCameraMatrix() << std::endl;
        std::cout << "Distortion coefficients:" << std::endl;
        std::cout << cameraCalibrator.getDistortionCoefficients() << std::endl;

		// find matching features between every 2 neighbor images 
        features.matchFeatures(cameraCalibrator, showMatchFeatures);
        std::cout << "Images features were matched" << std::endl;

		// get the features
        std::vector<imageFeatures> imagesFeatures = features.getFeatures();
        std::cout << "Starting structure from motion" << std::endl;;
        structureFromMotion structureFromMotion(cameraCalibrator, imagesFeatures);
        structureFromMotion.savePointCloud(objectImagesPath + "\\output\\PointCloud.ply");
        std::cout << "Structure from motion is finished" << std::endl;

        std::cout << "Starting surface reconstruction" << std::endl;
        surfaceReconstruction surfaceReconstruction(objectImagesPath + "\\output\\PointCloud.ply", objectImagesPath + "\\output\\Mesh.ply");

        std::cout << "Texturing mesh" << std::endl;
        meshTexturing::meshTexturing(imagesFeatures, cameraCalibrator, objectImagesPath + "\\output\\Mesh.ply", objectImagesPath + "\\output\\TexturMesh.obj", surfaceReconstruction.getPolygonMesh());
        std::cout << "Press any key to exit" << std::endl;
        std::getchar();
    }
    catch (const std::exception& e)
    {
        std::cout << "ERROR: " << e.what() << std::endl;
    }

    return 0;
}