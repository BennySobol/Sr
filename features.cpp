#include "features.h"
#include <opencv2\imgproc.hpp>
#include <opencv2\calib3d.hpp>


#define RATIO_THRESH 0.7


// detect and compute all of the images features
features::features(std::vector<std::string> images)
{
    cv::Ptr<cv::FeatureDetector> detector = cv::SIFT::create();
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
void features::matchFeatures()
{
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);

    for (int i = 0; i < _features.size() - 1; i++) {
        for (int j = i + 1; j < _features.size(); j++) {

            std::vector<std::vector<cv::DMatch>> matches;
            std::vector<cv::Point2f> points1, points2;
            cv::Mat mask;

            // 2 nearest neighbour match
            matcher->knnMatch(_features[i].descriptors, _features[j].descriptors, matches, 2); // k=2 therefore there will be two DMatch in matches[i]

            for (std::vector<cv::DMatch>& matche : matches) // the DMatches in matche vector are arranged in descending order of quality
            {
                //good matche is one with small distance measurement
                if (matche[0].distance < RATIO_THRESH * matche[1].distance)
                { // queryIdx refers to keypoints1 and trainIdx refers to keypoints2
                    points1.push_back(_features[i].keyPoints[matche[0].queryIdx].pt);
                    points2.push_back(_features[j].keyPoints[matche[0].trainIdx].pt);
                }
            }

            // erase bad matches using fundamental matrix constraint
            findFundamentalMat(points1, points2, cv::FM_RANSAC, 3.0, 0.99, mask);

            cv::Mat image;
            hconcat(_features[i].image, _features[j].image, image); // create image with image i and image j

            for (int k = mask.rows - 1; k >= 0 ; k--) {
                if (mask.at<uchar>(k, 0)) { // if the point was used to solve - value is 1 or 0 
                    // draw line on the image 
                    line(image, points1[k], points2[k] + cv::Point2f(_features[i].image.cols, 0), cv::Scalar(rand() % 255, rand() % 255, rand() % 255), 2);
                }
                else
                {
                    points1.erase(points1.begin() + k);
                    points2.erase(points2.begin() + k);
                }
            }

            _features[i].matchesKeyPoints[j] = { points1, points2 };

            std::cout << "Feature matching " << i << " / " << j << ", good matches " << points1.size() << std::endl;

            resize(image, image, image.size() / 2);// TO DO should be automatic
            imshow("img", image);
            cv::waitKey(100);
        }
    }
    cv::destroyAllWindows();
}


std::vector<imageFeatures>& features::getFeatures()
{
    return _features;
}