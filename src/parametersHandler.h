#pragma once
#include <iostream>
#include <string>
#include <vector>

class ParametersHandler
{
private:
	void printHelp();
public:
	bool _showMatch; // optional
	bool _isSorted; // optional
	double _downScaleFactor; // optional
	double _focalLength; // have to get a valid value (0 if he wants to use calib.xml)
	std::string _chessboardImagesPath; // chess board images for camera calibration - have to get a value - if he uses calib.xml value doesnt matter
	std::string _objectImagesPath; // folder with images of an object - requierd

	ParametersHandler(int argc, char* argv[]);
};