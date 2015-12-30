#pragma once

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace transformMaintenance
{

class transformMaintenance
{
public:
    const double PI = 3.1415926;
    const double rad2deg = 180 / PI;
    const double deg2rad = PI / 180;

    double timeOdomBefMapped;
    double timeOdomAftMapped;

    float transformSum[6] = {0};
    float transformIncre[6] = {0};
    float transformMapped[6] = {0};
    float transformBefMapped[6] = {0};
    float transformAftMapped[6] = {0};

    ros::Publisher *pubLaserOdometry2Pointer = NULL;
    tf::TransformBroadcaster *tfBroadcaster2Pointer = NULL;
    nav_msgs::Odometry laserOdometry2;
    tf::StampedTransform laserOdometryTrans2;

};

}
