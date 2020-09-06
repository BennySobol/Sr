#pragma once

#include <vector>
#include <set>
#include <iostream>

#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

class features
{
private:
	std::vector<std::pair<std::string, std::pair<std::vector<cv::KeyPoint>, cv::Mat>>> _features;
public:
	features(std::vector<std::string> images);
	std::vector<std::pair<std::string, std::pair<std::vector<cv::KeyPoint>, cv::Mat>>> getFeatures();
	void sortImages();
	std::pair<std::string, std::pair<std::vector<cv::KeyPoint>, cv::Mat>> mostSimiller(std::pair<std::string, std::pair<std::vector<cv::KeyPoint>, cv::Mat>> features);
};

