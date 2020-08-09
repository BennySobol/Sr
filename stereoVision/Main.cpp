#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace cv::xfeatures2d;
using std::cout;
using std::endl;

int main(int argc, char* argv[])
{
    Mat img1 = imread("img/1.jpg", IMREAD_COLOR);
    Mat img2 = imread("img/2.jpg", IMREAD_COLOR);
    if (img1.empty() || img2.empty())
    {
        cout << "Could not open or find the image!\n" << endl;
        return -1;
    }

    //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
    int minHessian = 300;
    Ptr<SURF> detector = SURF::create(minHessian);
    std::vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;
    detector->detectAndCompute(img1, noArray(), keypoints1, descriptors1);
    detector->detectAndCompute(img2, noArray(), keypoints2, descriptors2);
    //-- Step 2: Matching descriptor vectors with a FLANN based matcher
    // Since SURF is a floating-point descriptor NORM_L2 is used
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<DMatch> > knn_matches;
    matcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);
    //-- Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.7f;
    std::vector<DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
        }
    }
    cout << good_matches.size();
    //-- Draw matches
    Mat img_matches, resize;
    drawMatches(img1, keypoints1, img2, keypoints2, good_matches, img_matches, Scalar::all(-1),
        Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    //-- Show detected matches
    cv::resize(img_matches, resize, cv::Size(img_matches.size().width*0.45, img_matches.size().height * 0.45));

    imshow("Good Matches", resize);
    waitKey();
    return 0;
}
