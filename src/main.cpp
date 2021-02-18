#include "structureFromMotion.h"
#include "surfaceReconstruction.h"
#include "meshTexturing.h"
#include "loadImages.h"
#include "parametersHandler.h"


int main(int argc, char* argv[])
{
    try
    {
		ParametersHandler paramsHandler(argc, argv);
		
        fs::create_directories(paramsHandler._objectImagesPath + "\\output"); // create directories for output
        cameraCalibration cameraCalibrator;

		//load images
        std::vector<std::string> images = loadImages::getInstance()->load(paramsHandler._objectImagesPath, paramsHandler._isSorted);
        std::cout << "Images path were loaded\n";
		//extract features from every image
        features features(images, paramsHandler._downScaleFactor);

        std::cout << "Features were extracted\n";
        if (fs::exists(paramsHandler._objectImagesPath + "\\calib.xml")) // load calibration - if exsits
        {
            cameraCalibrator.load(paramsHandler._objectImagesPath + "\\calib.xml");
        }
        else if (paramsHandler._chessboardImagesPath != "") // calibrate camera
        {
            std::vector<std::string> ImagesPath = loadImages::getInstance()->load(paramsHandler._chessboardImagesPath, true);
            cameraCalibrator.addChessboardPoints(ImagesPath, cv::Size(9, 6)); // size of inner chessboard corners
            //print if there are errors, and save calibration file
            std::cout << "Error: " << cameraCalibrator.calibrate() << "\n";
            cameraCalibrator.save(paramsHandler._objectImagesPath + "\\calib.xml");
        }
        else if (paramsHandler._focalLength != 0) // cameraMatrix from focal length
        {
            cameraCalibrator.estimateCameraMatrix(paramsHandler._focalLength, features.getFeatures()[0].image.size());
        }
        else
        {
            throw std::exception("There mast be calibration parms");
        }
        cameraCalibrator.downScale(paramsHandler._downScaleFactor);

        // print camera params
        std::cout << "Calibration parms:" << std::endl;
        std::cout << "Camera matrix:" << std::endl;
        std::cout << cameraCalibrator.getCameraMatrix() << std::endl;
        std::cout << "Distortion coefficients:" << std::endl;
        std::cout << cameraCalibrator.getDistortionCoefficients() << std::endl;

		// find matching features between every 2 neighbor images 
        features.matchFeatures(cameraCalibrator, paramsHandler._showMatch);
        std::cout << "Images features were matched" << std::endl;

		// get the features
        std::vector<imageFeatures> imagesFeatures = features.getFeatures();
        std::cout << "Starting structure from motion" << std::endl; // there was another ';' in here - if  does error add again
        structureFromMotion structureFromMotion(cameraCalibrator, imagesFeatures);
        structureFromMotion.savePointCloud(paramsHandler._objectImagesPath + "\\output\\PointCloud.ply");
        std::cout << "Structure from motion is finished" << std::endl;

        std::cout << "Starting surface reconstruction" << std::endl;
        surfaceReconstruction surfaceReconstruction(paramsHandler._objectImagesPath + "\\output\\PointCloud.ply", paramsHandler._objectImagesPath + "\\output\\Mesh.ply");

        std::cout << "Texturing mesh" << std::endl;
        meshTexturing::meshTexturing(imagesFeatures, cameraCalibrator, paramsHandler._objectImagesPath + "\\output\\Mesh.ply", paramsHandler._objectImagesPath + "\\output\\TexturMesh.obj", surfaceReconstruction.getPolygonMesh());
        std::cout << "Press any key to exit" << std::endl;
        std::getchar();
    }
    catch (const std::exception& e)
    {
        std::cout << "ERROR: " << e.what() << std::endl;
    }

    return 0;
}