#pragma once
#include <vector>
#include <set>
#include <iostream>

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
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
	std::vector<std::string> load(const fs::path& dirPath);
	std::vector<std::string> getImages();
};