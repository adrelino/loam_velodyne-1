#pragma once
#include "ros/ros.h"
#include "ros/package.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"

#include "transformMaintenance.h"
#include "laserMapping.h"
#include "scanRegistration.h"
#include "laserOdometry.h"

class loam_wrapper
{
public:
    loam_wrapper();

    void publishInput(sensor_msgs::PointCloud2 &pc);

    ros::NodeHandle nh;
    ros::Publisher pubLaserOdometry2;
    ros::Publisher pubOdomBefMapped;
    ros::Publisher pubOdomAftMapped;
    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubLaserCloudExtreCur;
    ros::Publisher pubLaserCloudLast;
    ros::Publisher pubLaserOdometry;
    ros::Publisher pubLaserCloudLast2;
    ros::Publisher pubLaserInput;

    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform laserOdometryTrans;

    tf::TransformBroadcaster tfBroadcaster2;

    // Transform Maintance
    transformMaintenance::transformMaintenance * transformMain;

    // Laser Mapping
    laserMapping::laserMapping * laserMap;

    // Scan Registration
    scanRegistration::scanRegistration * scanReg;

    // Laser Odometry
    laserOdometry::laserOdometry * laserOd;

    sensor_msgs::PointCloud2 outExtreCur2;
    sensor_msgs::PointCloud2 outCloudLast2;

    sensor_msgs::PointCloud2 pub;
    nav_msgs::Odometry pubOdo;

    sensor_msgs::PointCloud2 laser_cloud_surround;
    nav_msgs::Odometry odomBefMapped;
    nav_msgs::Odometry odomAftMapped;

    nav_msgs::Odometry outlaserOdometry2;

    void newInPC(sensor_msgs::PointCloud2Ptr pc);
    void newInPCKITTI(sensor_msgs::PointCloud2Ptr pc, sensor_msgs::PointCloud2Ptr nextpc);

};
