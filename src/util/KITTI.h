#pragma once
#include <iostream>
#include <vector>
#include <dirent.h>
#include <sstream>
#include <fstream>
#include <algorithm>

using namespace std;


struct veloPoint{
    float x;
    float y;
    float z;
    float i;
};

class KITTI
{
public:
    KITTI();
    std::string path_to_image_0;
    std::string path_to_image_2;
    std::string path_to_velo;
    std::string pathPoses;

    std::vector<std::string> velo_files;
    std::vector<std::vector<veloPoint>> velpoints;

    int getVel(std::vector<std::string> &files, std::vector<std::vector<veloPoint> > &points, int num);
    int getFiles(std::string source, std::vector<std::string> &files);
    int getdir(std::string dir, std::vector<std::string> &files);
    int getFile (std::string source, std::vector<std::string> &files);
};
