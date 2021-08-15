#ifndef CONFIG_H
#define CONFIG_H

#include "common.h"

class Config
{
private:
    // private constructor ensures singleton pattern
    Config() {}
    // data members
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

public:
    ~Config();

    // set config file
    static bool setParameterFile(const std::string& filePath);
    // access parameters
    template <typename T>
    static T getParameter(const std::string& parameterName)
    {
        return T(Config::config_->file_[parameterName]);
    }
};

#endif // CONFIG_H