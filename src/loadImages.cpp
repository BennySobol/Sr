#include "loadImages.h"

// get loadImages Instance - a Singleton class
loadImages* loadImages::getInstance()
{
	static loadImages instance;

	return &instance;
}

// loading the images into vector
// return vector of strings - contains the images path
std::vector<std::string> loadImages::load(const fs::path& dirPath, bool isSorted)
{
	// filename extension to locate
	std::vector<fs::path> exts{ ".jpg", ".JPG", ".jpeg", ".JPEG", ".png", ".PNG" };
	// check if the path is ok
	if (!fs::exists(dirPath) || !fs::is_directory(dirPath))
	{
		throw std::runtime_error(dirPath.string() + " is not a valid folder");
	}
	// load the images names from the paths
	for (auto const& entry : fs::directory_iterator(dirPath))
	{
		if (fs::is_regular_file(entry) && std::find(exts.begin(), exts.end(), entry.path().extension()) != exts.end())
		{
			_images.push_back(entry.path().string());
		}
	}
	if (!isSorted) // if images are not already sorted by names (like pic1, pic2 ...)
	{
		features::sortImages(_images);
	}

	if (_images.size() < 2)
	{
		throw std::exception("There mast be at least 2 images in the folder");
	}
	return _images;
}

// loadImages destractor
loadImages::~loadImages()
{
	_images.clear();
}

// get the images paths
std::vector<std::string> loadImages::getImages()
{
	return _images;
}