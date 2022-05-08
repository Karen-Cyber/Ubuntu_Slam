#include <iostream>
#include <vector>
#include <string>
#include <cstring>
using namespace std;

#include <dirent.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

void get_name_list(string dir_path, vector<string>& name_list);

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        cerr << "usage: xml_conversion <image_dir_path> <output_dir_path>\n";
        exit(1);
    }
    if (!strcmp(argv[1], argv[2]))
    {
        cerr << "input directory can not be the same as output directroy\n";
        exit(3);
    }

    vector<string> name_list;
    get_name_list(string(argv[1]), name_list);

    // transform into grayscale
    string input_dir(argv[1]);
    string output_dir(argv[2]);
    for (const string& name : name_list)
    {
        cv::Mat grayscale = cv::imread(input_dir + "/" + name, cv::IMREAD_GRAYSCALE);
        cv::imwrite(output_dir + "/" + name, grayscale);
        // cout << output_dir + "/" + name << endl;
    }
    return 0;
}

void get_name_list(string dir_path, vector<string>& name_list)
{
    DIR* dir;
    if ((dir = opendir(dir_path.c_str())) == NULL)
    {
        cerr << "error opening directory...\n";
        exit(2);
    }

    dirent* entptr;
    while ((entptr = readdir(dir)) != NULL)
    {
        // current directory and upper directory 
        if (!strcmp(entptr->d_name, ".") || !strcmp(entptr->d_name, ".."))
            continue;
        // common file or link file
        if (entptr->d_type == 8 || 10)
            name_list.push_back(string(entptr->d_name));
        // sub-directory
        if (entptr->d_type == 4)
            get_name_list(dir_path.append("/").append(string(entptr->d_name)), name_list);
    }
}