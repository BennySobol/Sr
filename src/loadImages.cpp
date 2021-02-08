#include "loadImages.h"
#include "features.h"


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
    std::set<fs::path> exts{ ".jpg", ".JPG", ".jpeg", ".JPEG", ".png", ".PNG" };
	// check if the path is ok
    if (!fs::exists(dirPath) || !fs::is_directory(dirPath))
    {
       throw std::runtime_error(dirPath.string() + " is not a valid folder");
    }
	// load the images names from the paths
    for (auto const& entry : fs::directory_iterator(dirPath))
    {
        if (fs::is_regular_file(entry) && exts.find(entry.path().extension()) != exts.end())
        {
            _images.push_back(entry.path().string());
        }
    }
    if(!isSorted) // if images are not already sorted by names (like pic1, pic2 ...)
        sortImagesBySimilarity(); 

    if (_images.size() < 2)
    {
        throw std::exception("There mast be at least 2 images in the folder");
    }
    return _images;
}


std::map<std::string, cv::Mat> features;
cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);

// this function will sort _images vector by the images similarity
void loadImages::sortImagesBySimilarity()
{
    // find features using SIFT feature detector
    cv::Ptr<cv::FeatureDetector> detector = cv::SIFT::create();
    // find keypoints and descriptors of image using SIFT for every image in the vector
    // the detector with detect and compute are locating FEATURES and descriptors
    for (std::string path : _images)
    {
        cv::Mat src = cv::imread(path, cv::IMREAD_GRAYSCALE);
        resizeWithAspectRatio(src, 256);
        if (!src.empty())
        {
            cv::Mat descriptors;
            std::vector<cv::KeyPoint> keypoints;
            detector->detectAndCompute(src, cv::noArray(), keypoints, descriptors);
            features[path] = descriptors;
            std::cout << getFileNameWithExtension(path) << " " << keypoints.size() << " features were extracted\n";
        }
    }

    std::vector<std::string> sortedImages;
    int nextIndex = 0, size = _images.size();
    for (int i = 0; i < size; i++) {
        sortedImages.push_back(_images[nextIndex]);
        _images.erase(_images.begin() + nextIndex);
        if(_images.size() != 0)
            nextIndex = bestMatch(sortedImages[sortedImages.size()-1]);
    }
    _images = sortedImages;
}

// this function gets an image path and return the best matching index in _images vector
// gets image path to find match
// returns the best match image index
int loadImages::bestMatch(std::string path)
{
    double maxScore = 0, score = 0;
    int maxScoreIndex = 0;

    for (int i = 0; i < _images.size(); i++)
    {
        std::vector<std::vector<cv::DMatch>> matches;

        // 2 nearest neighbour match
        matcher->knnMatch(features[_images[i]], features[path], matches, 2); // k=2 therefore there will be two DMatch in matches[i]

        for (std::vector<cv::DMatch>& matche : matches) // the DMatches in matche vector are arranged in descending order of quality
        {
            //good matche is one with small distance measurement
            if (matche[0].distance < 0.7 * matche[1].distance)
            {
                score++;
            }
        }
        if (score > maxScore)
        {
            maxScore = score;
            maxScoreIndex = i;
        }
        score = 0;
    }
    // debug
    std::cout << getFileNameWithExtension(path) << " with " << getFileNameWithExtension(_images[maxScoreIndex]) << " are best match -> score " << maxScore << "\n";
    return maxScoreIndex;
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

std::string getFileNameWithExtension(std::string path)
{
    return path.substr(path.find_last_of("\\") + 1, path.size() - 1);
}

std::string getFileName(std::string path)
{
    std::string fileNameWithExtension = getFileNameWithExtension(path);
    return fileNameWithExtension.substr(0, fileNameWithExtension.find_first_of('.'));
}