#pragma once
#include <iostream>
#include <vector>

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "features.h"

#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

// Singleton designed
class loadImages
{
private:
	loadImages() = default;
	~loadImages();
	std::vector<std::string> _images;

public:
	static loadImages* getInstance();
	std::vector<std::string> load(const fs::path& dirPath, bool isSorted);
	std::vector<std::string> getImages();
};

