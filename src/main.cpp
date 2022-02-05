#include "structureFromMotion.h"
#include "surfaceReconstruction.h"
#include "meshTexturing.h"
#include "loadImages.h"
#include "parametersHandler.h"

#include <chrono> 

#pragma warning(disable:6031)

std::chrono::high_resolution_clock::time_point timer;

void clockHandle()
{
	std::cout << "In " << (double)std::chrono::duration_cast<std::chrono::milliseconds>
		((std::chrono::high_resolution_clock::now() - timer)).count() / 600 << " seconds" << std::endl << std::endl;
	timer = std::chrono::high_resolution_clock::now();
}

void handleCalibrationParms(cameraCalibration& calib, const parametersHandler paramsHandler, const std::string fistImagePath)
{
	// handle calibration parms
	if (fs::exists(paramsHandler.objectImagesPath + "\\calib.xml")) // load calibration - if exsits
	{
		std::cout << "Using calib.xml as calibration parms" << std::endl;
		calib.load(paramsHandler.objectImagesPath + "\\calib.xml");
	}
	else if (paramsHandler.chessboardImagesPath != "") // calibrate camera
	{
		std::cout << "Camera calibration using chessboard images at " << paramsHandler.chessboardImagesPath << std::endl;
		std::vector<std::string> chessboardImages = loadImages::getInstance()->load(paramsHandler.chessboardImagesPath);
		calib.addChessboardPoints(chessboardImages, cv::Size(9, 6), paramsHandler.showMatch); // size of inner chessboard corners
		// print if there are errors, and save calibration file
		std::cout << "Error: " << calib.calibrate() << std::endl;
		calib.save(paramsHandler.objectImagesPath + "\\calib.xml");
	}
	else if (paramsHandler.focalLength != 0) // cameraMatrix from focal length
	{
		std::cout << "Using focal length as calibration parms" << std::endl;
		calib.estimateCameraMatrix(paramsHandler.focalLength, cv::imread(fistImagePath).size());
	}
	else
	{
		throw std::runtime_error("There mast be calibration parms");
	}
	calib.downScale(paramsHandler.downScaleFactor);
}

int main(int argc, char* argv[])
{
	std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now();
	timer = startTime;
	try
	{
		parametersHandler paramsHandler(argc, argv);
		cameraCalibration calib;

		// load images
		std::vector<std::string> images = loadImages::getInstance()->load(paramsHandler.objectImagesPath, paramsHandler.isSorted);
		std::cout << "Images path were loaded ";
		clockHandle();

		handleCalibrationParms(calib, paramsHandler, images[0]);
		std::cout << calib; // print camera params
		clockHandle();

		// create directories for output
		fs::create_directories(paramsHandler.objectImagesPath + "\\output");

		// extract features from every image
		features features(images, paramsHandler.downScaleFactor);

		std::cout << "Features were extracted ";
		clockHandle();

		// find matching features between every 2 neighbor images 
		features.matchAllFeatures(paramsHandler.showMatch);
		std::cout << "Images features were matched ";
		clockHandle();

		std::cout << "Starting structure from motion" << std::endl;
		structureFromMotion structureFromMotion(0.35);
		structureFromMotion.savePointCloud(paramsHandler.objectImagesPath + "\\output\\PointCloud.ply");
		std::cout << "Structure from motion is finished ";
		clockHandle();

		std::cout << "Starting surface reconstruction" << std::endl;
		surfaceReconstruction surfaceReconstruction(paramsHandler.objectImagesPath + "\\output\\PointCloud.ply", paramsHandler.objectImagesPath + "\\output\\Mesh.ply");
		clockHandle();

		std::cout << "Texturing mesh" << std::endl;
		meshTexturing meshTexturing(paramsHandler.objectImagesPath + "\\output\\Mesh.ply", paramsHandler.objectImagesPath + "/output/TextureMesh.obj", surfaceReconstruction.getPolygonMesh());
		clockHandle();
		std::cout << "Total time is " << (double)std::chrono::duration_cast<std::chrono::milliseconds>
			((std::chrono::high_resolution_clock::now() - startTime)).count() / 600 << " seconds" << std::endl;
		std::cout << "Press any key to exit" << std::endl;
		getchar();
	}
	catch (const std::exception& e)
	{
		std::cout << "ERROR: " << e.what() << std::endl;
	}

	return 0;
}