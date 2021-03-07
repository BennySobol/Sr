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

		cameraCalibration calib;

		//load images
		std::vector<std::string> images = loadImages::getInstance()->load(paramsHandler.objectImagesPath, paramsHandler.isSorted);
		std::cout << "Images path were loaded\n";

		// create directories for output
		fs::create_directories(paramsHandler.objectImagesPath + "\\output");

		//extract features from every image
		features features(images, paramsHandler.downScaleFactor);

		std::cout << "Features were extracted\n";
		if (fs::exists(paramsHandler.objectImagesPath + "\\calib.xml")) // load calibration - if exsits
		{
			calib.load(paramsHandler.objectImagesPath + "\\calib.xml");
		}
		else if (paramsHandler.chessboardImagesPath != "") // calibrate camera
		{
			std::vector<std::string> ImagesPath = loadImages::getInstance()->load(paramsHandler.chessboardImagesPath, true);
			calib.addChessboardPoints(ImagesPath, cv::Size(9, 6)); // size of inner chessboard corners
			//print if there are errors, and save calibration file
			std::cout << "Error: " << calib.calibrate() << "\n";
			calib.save(paramsHandler.objectImagesPath + "\\calib.xml");
		}
		else if (paramsHandler.focalLength != 0) // cameraMatrix from focal length
		{
			calib.estimateCameraMatrix(paramsHandler.focalLength, features.getFeatures()[0].image.size());
		}
		else
		{
			throw std::exception("There mast be calibration parms");
		}
		calib.downScale(paramsHandler.downScaleFactor);

		// print camera params
		std::cout << "Calibration parms:" << std::endl;
		std::cout << "Camera matrix:" << std::endl;
		std::cout << calib.getCameraMatrix() << std::endl;
		std::cout << "Distortion coefficients:" << std::endl;
		std::cout << calib.getDistortionCoefficients() << std::endl;

		// find matching features between every 2 neighbor images 
		features.matchAllFeatures(calib, paramsHandler.showMatch);
		std::cout << "Images features were matched" << std::endl;

		// get the features
		std::vector<imageFeatures> imagesFeatures = features.getFeatures();
		std::cout << "Starting structure from motion" << std::endl; // there was another ';' in here - if  does error add again
		structureFromMotion structureFromMotion(calib, imagesFeatures);
		structureFromMotion.savePointCloud(paramsHandler.objectImagesPath + "\\output\\PointCloud.ply");
		std::cout << "Structure from motion is finished" << std::endl;

		std::cout << "Starting surface reconstruction" << std::endl;
		surfaceReconstruction surfaceReconstruction(paramsHandler.objectImagesPath + "\\output\\PointCloud.ply", paramsHandler.objectImagesPath + "\\output\\Mesh.ply");

		std::cout << "Texturing mesh" << std::endl;
		meshTexturing(imagesFeatures, calib, paramsHandler.objectImagesPath + "\\output\\Mesh.ply", paramsHandler.objectImagesPath + "/output/TextureMesh.obj", surfaceReconstruction.getPolygonMesh());
		std::cout << "Press any key to exit" << std::endl;
		getchar();
	}
	catch (const std::exception& e)
	{
		std::cout << "ERROR: " << e.what() << std::endl;
	}

	return 0;
}