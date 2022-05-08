#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstring>
using namespace std;

#include <dirent.h>
#include <unistd.h>

void get_name_list(string dir_path, vector<string>& name_list);

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        cerr << "usage: xml_conversion <image_dir_path> <output_file_name>\n";
        exit(1);
    }

    vector<string> name_list;
    get_name_list(string(argv[1]), name_list);

    ofstream outFile(argv[2]);

    outFile << "<?xml version=\"1.0\"?>\n";
    outFile << "<opencv_storage>\n";
    outFile << "\t<images>\n";
    for (const string& path : name_list)
        outFile << "\t\t" << '\"' << path << '\"' << endl;
    outFile << "\t</images>\n";
    outFile << "</opencv_storage>\n";


    outFile.close();
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
        string absolute_path = dir_path;
        // current directory and upper directory 
        if (!strcmp(entptr->d_name, ".") || !strcmp(entptr->d_name, ".."))
            continue;
        // common file or link file
        if (entptr->d_type == 8 || 10)
        {
            absolute_path.append("/").append(string(entptr->d_name));
            name_list.push_back(absolute_path);
            cout << absolute_path << endl;
        }
        // sub-directory
        if (entptr->d_type == 4)
        {
            get_name_list(dir_path.append("/").append(string(entptr->d_name)), name_list);
        }
    }
}