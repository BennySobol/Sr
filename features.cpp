#include "features.h"

features::features(std::vector<std::string> images)
{
    int minHessian = 400;
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(minHessian);
    for (std::string image : images)
    {
        cv::Mat src = cv::imread(image);
        if (!src.empty())
        {
            cv::Mat descriptors;
            std::vector<cv::KeyPoint> keypoints;
            detector->detectAndCompute(src, cv::noArray(), keypoints, descriptors);
            _features.push_back({ image , {keypoints, descriptors} });
        }
    }
}

std::vector<std::pair<std::string, std::pair<std::vector<cv::KeyPoint>, cv::Mat>>> features::getFeatures()
{
    return _features;
}

void features::sortImages()
{
    std::vector<std::pair<std::string, std::pair<std::vector<cv::KeyPoint>, cv::Mat>>> sortedFeatures;
    sortedFeatures.push_back(_features[0]);
    _features.erase(_features.begin());
    while(_features.size() != 1)
    {
        std::pair<std::string, std::pair<std::vector<cv::KeyPoint>, cv::Mat>> next = mostSimiller(sortedFeatures.back());
        sortedFeatures.push_back(next);
        _features.erase(std::find_if(_features.begin(), _features.end(), [&next](std::pair<std::string, std::pair<std::vector<cv::KeyPoint>, cv::Mat>>& obj) { return obj.first == next.first; }));
    }
    sortedFeatures.push_back(_features.back());
    _features = sortedFeatures;
}	

std::pair<std::string, std::pair<std::vector<cv::KeyPoint>, cv::Mat>> features::mostSimiller(std::pair<std::string, std::pair<std::vector<cv::KeyPoint>, cv::Mat>> features)
{
    std::pair<std::string, std::pair<std::vector<cv::KeyPoint>, cv::Mat>> similler;
    int maxGoodMatches = 0;
    for (std::pair<std::string, std::pair<std::vector<cv::KeyPoint>, cv::Mat>> feature2 : _features)
    {
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        std::vector<std::vector<cv::DMatch> > knnMatches;
        matcher->knnMatch(features.second.second, feature2.second.second, knnMatches, 2);
        //-- Filter matches using the Lowe's ratio test
        const float ratio_thresh = 0.7;
        std::vector<cv::DMatch> goodMatches;
        for (size_t i = 0; i < knnMatches.size(); i++)
        {
            if (knnMatches[i][0].distance < ratio_thresh * knnMatches[i][1].distance)
            {
                goodMatches.push_back(knnMatches[i][0]);
            }
        }
        if (goodMatches.size() > maxGoodMatches)
        {
            maxGoodMatches = goodMatches.size();
            similler = feature2;
        }
    }
    return similler;
}