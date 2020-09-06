#include "loadImages.h"
#include "features.h"
#include <iostream>


int main(int argc, char* argv[])
{
    auto images = loadImages::getInstance()->load("C:\\img");
    features features(images);
    // features.sortImages(); //This function is so timewise expensive
    auto imagesFeatures = features.getFeatures();
    for (auto imageFeatures : imagesFeatures)
    {
        cv::Mat imgKeypoints;
        drawKeypoints(cv::imread(imageFeatures.first), imageFeatures.second.first, imgKeypoints);
        imshow("SURF Keypoints", imgKeypoints);
        cv::waitKey(); // press any key for next image
    }
    return 0;
}