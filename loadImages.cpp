#include "loadImages.h"

// get loadImages Instance - a Singleton class
loadImages* loadImages::getInstance()
{
    static loadImages instance;

    return &instance;
}

//loading the images into vector
//return vector of strings - contains omages names
std::vector<std::string> loadImages::load(const fs::path& dirPath, bool isSorted)
{
	//pics formats to locate
    std::set<fs::path> exts{ ".jpg", ".jpeg", ".png", ".JPG" };
	//check if path is ok
    if (!fs::exists(dirPath) || !fs::is_directory(dirPath))
    {
        throw std::runtime_error(dirPath.string() + " is not a valid folder");
    }
	//load the images paths
    for (auto const& entry : fs::recursive_directory_iterator(dirPath))
    {
        if (fs::is_regular_file(entry) && exts.find(entry.path().extension()) != exts.end())
        {
            _images.push_back(entry.path().string());
        }
    }
    if(!isSorted) // if images are not sorted by names (like pic1, pic2 ...) than sort by similarity
        sortImagesBySimilarity();
    if (_images.size() < 2)
    {
        throw std::exception("There mast be at least 2 images in the folder");
    }
    return _images;
}


// this function will sort _images vector by the images similarity
//
void loadImages::sortImagesBySimilarity()
{
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
//geys image path to find match
//returns the best match image index
int loadImages::bestMatch(std::string path)
{
    cv::Mat image_i, image;
    cv::Mat scoreImg;
    double maxScore = 0, score;
    int maxScoreIndex = 0;

    image = cv::imread(path, cv::IMREAD_COLOR);
    if (image.empty())
        return -1;

    for (int i = 0; i < _images.size(); i++) {

        image_i = cv::imread(_images[i], cv::IMREAD_COLOR); // loads image itself
        if (image_i.empty())
            continue;

        // claculating the similarity between the images using cv functions
        cv::matchTemplate(image_i, image, scoreImg, cv::TM_CCOEFF_NORMED);
        cv::minMaxLoc(scoreImg, 0, &score);
        if (score > maxScore)
        {
            maxScore = score;
            maxScoreIndex = i;
        }
    }
    // debug
    std::cout << path.substr(path.find_last_of("\\") + 1, path.size() - 1) << " with " << _images[maxScoreIndex].substr(_images[maxScoreIndex].find_last_of("\\") + 1, _images[maxScoreIndex].size() - 1)  << " are best match -> score " << maxScore << "\n";
    return maxScoreIndex;
}

//destractor
loadImages::~loadImages()
{
    _images.clear();
}
//get the images paths
std::vector<std::string> loadImages::getImages()
{
    return _images;
}