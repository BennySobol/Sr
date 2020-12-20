#include "features.h"
#include <opencv2\imgproc.hpp>
#include <opencv2\calib3d.hpp>


#define RATIO_THRESH 0.7


// detect and compute all of the images features
features::features(std::vector<std::string> images)
{
    cv::Ptr<cv::FeatureDetector> detector = cv::SIFT::create(); //find features using SIFT algorithm/method
	//find keypoints and descriptors of image using SIFT for every image in the vector
	//the detector with detect and compute are locating FEATURES and descriptors
    for (std::string image : images)
    {
        cv::Mat src = cv::imread(image);
        if (!src.empty())
        {
            cv::Mat descriptors;
            std::vector<cv::KeyPoint> keypoints;
            detector->detectAndCompute(src, cv::noArray(), keypoints, descriptors);
            _features.push_back(imageFeatures{ image, src, keypoints, descriptors });
            std::cout << image << " " << keypoints.size() <<" features were extracted\n";
        }
    }
}



// match the features between the images
//NOTE: ReCheck for understanding!
void features::matchFeatures(cameraCalibration calib)
{
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
	//check for all features found
    for (int i = 0; i < _features.size() - 1; i++) {

        std::vector<std::vector<cv::DMatch>> matches;
        std::vector<cv::Point2f> points1, points2;
        std::vector<int> points1Idx, points2Idx;
        cv::Mat mask;

        // 2 nearest neighbour match
        matcher->knnMatch(_features[i].descriptors, _features[i+1].descriptors, matches, 2); // k=2 therefore there will be two DMatch in matches[i]

        for (std::vector<cv::DMatch>& matche : matches) // the DMatches in matche vector are arranged in descending order of quality
        {
            //good matche is one with small distance measurement
            if (matche[0].distance < RATIO_THRESH * matche[1].distance)
            { 
                // queryIdx refers to keypoints1 and trainIdx refers to keypoints2
                points1.push_back(_features[i].keyPoints[matche[0].queryIdx].pt);
                points2.push_back(_features[i + 1].keyPoints[matche[0].trainIdx].pt);
                points1Idx.push_back(matche[0].queryIdx);
                points2Idx.push_back(matche[0].trainIdx);
            }
        }

        // erase bad matches using fundamental matrix constraint
        cv::Mat E = findEssentialMat(points1, points2, calib.getFocal(), calib.getPP(), cv::RANSAC, 0.99, 1.0, mask);

        cv::Mat image;
        hconcat(_features[i].image, _features[i + 1].image, image); // create image with image i and image j

        for (int j = mask.rows - 1; j >= 0 ; j--) {
            if (mask.at<uchar>(j, 0)) { // if the point was used to solve - value is 1 or 0 
                // draw line on the image 
                line(image, points1[j], points2[j] + cv::Point2f(_features[i].image.cols, 0), cv::Scalar(rand() % 255, rand() % 255, rand() % 255), 2);
            }
            else
            {
                points1.erase(points1.begin() + j);
                points2.erase(points2.begin() + j);
                points1Idx.erase(points1Idx.begin() + j);
                points2Idx.erase(points2Idx.begin() + j);
            }
        }

        _features[i].matchingKeyPoints = { points1, points2, points1Idx, points2Idx };

        std::cout << "Feature matching " << i << " / " << i + 1 << ", good matches " << points1.size() << std::endl;

        resize(image, image, image.size() / 4);// TO DO should be automatic
        imshow("Feature matching", image);
        cv::waitKey(100);
    }
    cv::destroyAllWindows();
}

//return the features vector
std::vector<imageFeatures>& features::getFeatures()
{
    return _features;
}