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

class loam_wrapper
{
public:
    loam_wrapper();

    void publishInput(sensor_msgs::PointCloud2 &pc);
    void publishGT(nav_msgs::Odometry &od);
    void publishLaserCloudLast2(sensor_msgs::PointCloud2 &pc);
    void publishLaserCloudLast(sensor_msgs::PointCloud2 &pc);
    void publishLaserCloudLast2(pcl::PointCloud<pcl::PointXYZHSV>::Ptr pc);

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
    ros::Publisher pubGT;

    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform laserOdometryTrans;

    tf::TransformBroadcaster tfBroadcaster2;

    // Transform Maintance
    transformMaintenance::transformMaintenance * transformMain;

    // Laser Mapping
    laserMapping::laserMapping * laserMap;

    // Scan Registration
    scanRegistration::scanRegistration * scanReg;
    scanRegistration::scanRegistration * scanReg2;

    // Laser Odometry
    laserOdometry::laserOdometry * laserOd;

    sensor_msgs::PointCloud2 pub;
    nav_msgs::Odometry pubOdo;

    sensor_msgs::PointCloud2 laser_cloud_surround;
    nav_msgs::Odometry odomBefMapped;
    nav_msgs::Odometry odomAftMapped;

    nav_msgs::Odometry outlaserOdometry2;

    void newInPC(sensor_msgs::PointCloud2Ptr pc);
    void newInPCKITTI(sensor_msgs::PointCloud2 &pc, sensor_msgs::PointCloud2 &nextpc, Eigen::Matrix4d T_gt);
    void mappingTest(sensor_msgs::PointCloud2 &pc, sensor_msgs::PointCloud2 &nextpc, Eigen::Matrix4d T);


    nav_msgs::Odometry gt;

    void publishInput(pcl::PointCloud<pcl::PointXYZHSV>::Ptr pc);

    Eigen::Matrix4d T_total;
    Eigen::Matrix4d T_gt;

};
