#pragma once
#include <iostream>
#include <string>
#include <vector>

class parametersHandler
{
private:
	void printHelp();
public:
	bool showMatch;
	bool isSorted;
	double downScaleFactor;
	double focalLength; // 0 if user has calib.xml
	std::string chessboardImagesPath; // chess board images for camera calibration
	std::string objectImagesPath; // folder with images of an object - requierd

	parametersHandler(int argc, char* argv[]);
};