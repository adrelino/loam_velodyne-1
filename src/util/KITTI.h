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
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/search/organized.h>

#include <pcl/filters/passthrough.h>

#include <pcl/common/common.h>

#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"
#include "tf2_msgs/TFMessage.h"

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

    int getVel(std::vector<std::string> &files, std::vector<std::vector<veloPoint> > &points, int num, int start);
    int getFiles(std::string source, std::vector<std::string> &files);
    int getdir(std::string dir, std::vector<std::string> &files);
    int getFile (std::string source, std::vector<std::string> &files);
    void getPointCloud2(sensor_msgs::PointCloud2 & outPC, int i);

    Eigen::Matrix4d getVelo_to_cam_T();

    int getGtCameraPoses(std::vector<Eigen::Matrix3d> &Rs, std::vector<Eigen::Vector3d> &ts);
    void getGtCameraPoses(std::vector<Eigen::Matrix4d> &Ts);
    void getGtCameraPosesAsNavMsg(std::vector<nav_msgs::Odometry> &out);
    void getPointCloud(pcl::PointCloud<pcl::PointXYZ> & msg, int i);
    void getVeloPC(pcl::PointCloud<pcl::PointXYZ> & msg, std::vector<veloPoint> & veloPoints);
    void writeResult(std::vector<Eigen::Matrix4d> Ts);
    int getOneVel(std::vector<veloPoint> &points, int j);
private:
    void createPointCloud2(sensor_msgs::PointCloud2 & outPC, std::vector<veloPoint> & veloPoints);
    Eigen::Matrix3d getVelo_to_cam_R();
    Eigen::Vector3d getVelo_to_cam_t();

};
