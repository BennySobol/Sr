#pragma once
#include <iostream>
#include <string>
#include <vector>

class ParametersHandler
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

	ParametersHandler(int argc, char* argv[]);
};