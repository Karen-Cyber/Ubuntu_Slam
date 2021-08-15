#include "config.h"

bool Config::setParameterFile(const std::string& filePath)
{
    if (config_ == nullptr)
    {
        config_ = std::shared_ptr<Config>(new Config);
        config_->file_ = cv::FileStorage(filePath.c_str(), cv::FileStorage::READ);
        if (!config_->file_.isOpened())
        {
            std::cout << "parameter file " << filePath << " does not exist\n";
            config_->file_.release();
            return false;
        }
    }
    return true;
}

Config::~Config()
{
    if (file_.isOpened())
        file_.release();
}

std::shared_ptr<Config> Config::config_ = nullptr;