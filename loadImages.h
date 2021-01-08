#pragma once
#include <vector>
#include <set>
#include <iostream>
#include <boost/filesystem.hpp>

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"


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
	std::vector<std::string> load(const fs::path& dirPath, bool isSorted);
	std::vector<std::string> getImages();
	void sortImagesBySimilarity();
	int bestMatch(std::string path);
};
