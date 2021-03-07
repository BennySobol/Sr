#include "features.h"
#include "wtypes.h"


#pragma warning(disable:26451)

#define RATIO_THRESH 0.7

std::vector<imageFeatures> features::_features;
cv::Ptr<cv::FeatureDetector> features::_detector = cv::SIFT::create();
cv::Ptr<cv::DescriptorMatcher> features::_matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);

// detect and compute all of the images features
features::features(std::vector<std::string> images, double downScale)
{
	// find features using SIFT feature detector
	// find keypoints and descriptors of image using SIFT for every image in the vector
	// the detector with detect and compute are locating FEATURES and descriptors
	for (std::string path : images)
	{
		_features.push_back(extractFeatures(path, downScale));
	}
}

void features::matchFeatures(imageFeatures& firstImageFeatures, imageFeatures& secondImageFeatures,
	std::vector<cv::Point2f>& firstPoints, std::vector<cv::Point2f>& secondPoints)
{
	std::vector<std::vector<cv::DMatch>> matches;
	std::vector<int>& firstPointsIdx = firstImageFeatures.matchingKeyPoints.currentKeyPointsIdx;
	std::vector<int>& secondPointsIdx = firstImageFeatures.matchingKeyPoints.otherKeyPointsIdx;
	// 2 nearest neighbour match
	_matcher->knnMatch(firstImageFeatures.descriptors, secondImageFeatures.descriptors, matches, 2); // k=2 therefore there will be two DMatch in matches[i]

	for (std::vector<cv::DMatch>& matche : matches) // the DMatches in matche vector are arranged in descending order of quality
	{
		// good matche is one with small distance measurement
		if (matche[0].distance < RATIO_THRESH * matche[1].distance)
		{
			// queryIdx refers to keypoints1 and trainIdx refers to keypoints2
			firstPoints.push_back(firstImageFeatures.keyPoints[matche[0].queryIdx].pt);
			secondPoints.push_back(secondImageFeatures.keyPoints[matche[0].trainIdx].pt);
			firstPointsIdx.push_back(matche[0].queryIdx);
			secondPointsIdx.push_back(matche[0].trainIdx);
		}
	}
}


// match the features between the images
void features::matchAllFeatures(cameraCalibration calib, bool showMatchingFeature)
{
	int screenWidth = getScreenWidth();

	//check for all features found
	for (unsigned int i = 0; i < _features.size() - 1; i++)
	{
		std::vector<cv::Point2f> firstPoints, secondPoints;
		std::vector<int>& firstPointsIdx = _features[i].matchingKeyPoints.currentKeyPointsIdx;
		std::vector<int>& secondPointsIdx = _features[i].matchingKeyPoints.otherKeyPointsIdx;

		cv::Mat image, mask;
		matchFeatures(_features[i], _features[i + 1], firstPoints, secondPoints);
		if (showMatchingFeature)
		{
			hconcat(_features[i].image, _features[i + 1].image, image); // create image with image i and image  j
		}

		// erase bad matches using fundamental matrix constraints
		cv::Mat E = findEssentialMat(firstPoints, secondPoints, calib.getCameraMatrix(), cv::RANSAC, 0.9999999999999999, 1.0, mask);

		for (int j = mask.rows - 1; j >= 0; j--)
		{
			if (mask.at<uchar>(j, 0)) // if the point was used to solve the essential matrix - the value is 1 otherwise 0 
			{
				if (showMatchingFeature)
				{
					// draw line on the image 
					line(image, firstPoints[j], secondPoints[j] + cv::Point2f(_features[i].image.cols, 0), cv::Scalar(rand() % 255, rand() % 255, rand() % 255), 2);
				}
			}
			else
			{
				firstPointsIdx.erase(firstPointsIdx.begin() + j);
				secondPointsIdx.erase(secondPointsIdx.begin() + j);
			}
		}

		if (_features[i].matchingKeyPoints.currentKeyPointsIdx.size() < 20)
		{
			_features.erase(_features.begin() + i, _features.end());
			break;
		}
		std::cout << "Feature matching " << getFileName(_features[i].path) << " / " << getFileName(_features[i + 1].path) << ", good matches " << firstPointsIdx.size() << std::endl;
		if (showMatchingFeature)
		{
			resizeWithAspectRatio(image, screenWidth);
			cv::imshow("Feature matching", image);
			cv::waitKey(100);
		}
	}
	cv::destroyAllWindows();
}

void features::getCurrentKeyPoints(std::vector<cv::Point2f>& currentKeyPoints, int featureIndex)
{
	currentKeyPoints.clear();
	for (int index : _features[featureIndex].matchingKeyPoints.currentKeyPointsIdx)
	{
		currentKeyPoints.push_back(_features[featureIndex].keyPoints[index].pt);
	}
}

void features::getOtherKeyPoints(std::vector<cv::Point2f>& otherKeyPoints, int featureIndex)
{
	otherKeyPoints.clear();
	for (int index : _features[featureIndex].matchingKeyPoints.otherKeyPointsIdx)
	{
		otherKeyPoints.push_back(_features[featureIndex + 1].keyPoints[index].pt);
	}
}

int features::matchFeaturesScore(cv::Mat firstImageDescriptors, cv::Mat secondImageDescriptors)
{
	int score = 0;
	std::vector<std::vector<cv::DMatch>> matches;
	// 2 nearest neighbour match
	_matcher->knnMatch(firstImageDescriptors, secondImageDescriptors, matches, 2); // k=2 therefore there will be two DMatch in matches[i]

	// the DMatches in matche vector are arranged in descending order of quality
	for (std::vector<cv::DMatch>& matche : matches)
	{
		// good matche is one with small distance measurement
		if (matche[0].distance < RATIO_THRESH * matche[1].distance)
		{
			score++;
		}
	}
	return score;
}


// function to find the maximum cost path for all possible the paths
std::vector<int> features::findMaxRoute(const std::vector<std::vector<int>> scoreMatrix)
{
	int sum = 0;
	int counter = 0;
	int j = 0, i = 0;
	int max = INT_MIN;
	std::map<int, int> visitedRouteList;

	// Starting from the 0th indexed
	visitedRouteList[0] = 1;
	std::vector<int> route(scoreMatrix.size());

	// traverse the adjacency matrix - scoreMatrix
	while (i < scoreMatrix.size() && j < scoreMatrix[i].size())
	{
		// corner of the matrix
		if (counter >= scoreMatrix[i].size() - 1)
		{
			break;
		}

		// if this path is unvisited then and if the cost is more then update the cost
		if (j != i && (visitedRouteList[j] == 0))
		{
			if (scoreMatrix[i][j] > max)
			{
				max = scoreMatrix[i][j];
				route[counter] = j + 1;
			}
		}
		j++;

		// Check all paths from the ith indexed city
		if (j == scoreMatrix[i].size())
		{
			sum += max;
			max = INT_MIN;
			visitedRouteList[route[counter] - 1] = 1;
			j = 0;
			i = route[counter] - 1;
			counter++;
		}
	}

	// update the ending node in array from node which was last visited
	i = route[counter - 1] - 1;

	for (j = 0; j < scoreMatrix.size(); j++)
	{

		if ((i != j) && scoreMatrix[i][j] > max)
		{
			max = scoreMatrix[i][j];
			route[counter] = j + 1;
		}
	}
	sum += max;

	// started from the node where we finished as well.
	std::cout << "Max score sum is : " << sum << std::endl;
	for (int i = 0; i < scoreMatrix.size(); i++)
	{
		std::cout << route[i] - 1 << " ";
	}
	std::cout << std::endl;
	return route;
}

void features::sortImages(std::vector<std::string>& images)
{
	std::map<std::string, cv::Mat> descriptors;
	int size = 0;
	for (int i = 1; i < 5; i++)
	{
		size += (widthFor900Features(images[(images.size() / 5) + i - 1]) - size) / i;
		std::cout << size << std::endl;
	}
	for (std::string path : images)
	{
		descriptors[path] = smallExtractFeatures(path, size);
	}

	std::vector<std::vector<int>> scoreMatrix;
	for (int i = 0; i < images.size(); i++)
	{
		scoreMatrix.push_back(std::vector<int>(images.size()));
	}
	for (int i = 0; i < images.size(); i++)
	{
		for (int j = i + 1; j < images.size(); j++)
		{
			int score = matchFeaturesScore(descriptors[images[i]], descriptors[images[j]]);
			scoreMatrix[i][j] = score;
			scoreMatrix[j][i] = score;
		}
		std::cout << "matching " << i << " / " << images.size() - 1 << std::endl;
	}

	std::vector<int> sorted = findMaxRoute(scoreMatrix);

	// sorted images according to max routes ordering
	std::vector<std::string> sortedImages;
	for (int i = 0; i < images.size(); i++)
	{
		sortedImages.push_back(images[sorted[i] - 1]);
	}
	images = sortedImages;
}

// return the features vector
std::vector<imageFeatures>& features::getFeatures()
{
	return _features;
}

// resize image height to be with aspect ratio with a given width
void resizeWithAspectRatio(cv::Mat& image, int width)
{
	float ratio = width / float(image.size().width);
	cv::Size dim = cv::Size(width, int(image.size().height * ratio));

	resize(image, image, dim);
}

// get the screen width
int getScreenWidth()
{
	RECT desktop;
	// Get a handle to the desktop window
	const HWND hDesktop = GetDesktopWindow();
	// Get the size of screen to the variable desktop
	GetWindowRect(hDesktop, &desktop);

	return desktop.right; //screen width
}

std::string getFileNameWithExtension(std::string path)
{
	return path.substr(path.find_last_of("\\") + 1, path.size() - 1);
}

std::string getFileName(std::string path)
{
	std::string fileNameWithExtension = getFileNameWithExtension(path);
	return fileNameWithExtension.substr(0, fileNameWithExtension.find_first_of('.'));
}