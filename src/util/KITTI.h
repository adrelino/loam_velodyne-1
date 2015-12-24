#pragma once
#include <iostream>
#include <vector>
#include <dirent.h>
#include <sstream>
#include <fstream>
#include <algorithm>

#include <pcl/point_cloud.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/impl/point_types.hpp>

#include "sensor_msgs/PointCloud2.h"

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
    void getPointCloud2(sensor_msgs::PointCloud2 & outPC, int i);
private:
    void createPointCloud2(sensor_msgs::PointCloud2 & outPC, std::vector<veloPoint> & veloPoints);
};
