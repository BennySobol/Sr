#pragma once
#include <iostream>
#include <vector>

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "features.h"

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

// Singleton designed
class loadImages
{
private:
	loadImages() = default;
	~loadImages();
	std::vector<std::string> _images;

public:
	static loadImages* getInstance();
	std::vector<std::string> load(const fs::path& dirPath, const bool isSorted = true);
	std::vector<std::string> getImages();
};

