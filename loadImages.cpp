#include "loadImages.h"

// get loadImages Instance - a Singleton class
loadImages* loadImages::getInstance()
{
    static loadImages instance;

    return &instance;
}


std::vector<std::string> loadImages::load(const fs::path& dirPath)
{
    std::set<fs::path> exts{ ".jpg", ".jpeg", ".png" };

    if (!fs::exists(dirPath) || !fs::is_directory(dirPath))
    {
        return {};
    }

    for (auto const& entry : fs::recursive_directory_iterator(dirPath))
    {
        if (fs::is_regular_file(entry) && exts.find(entry.path().extension()) != exts.end())
        {
            _images.push_back(entry.path().string());
        }
    }
    return _images;
}


loadImages::~loadImages()
{
    _images.clear();
}


std::vector<std::string> loadImages::getImages()
{
    return _images;
}