#include "ros/ros.h"
#include "ros/package.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "nav_msgs/Odometry.h"
#include <boost/foreach.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include "cv_bridge/cv_bridge.h"
#define foreach BOOST_FOREACH

#include "loam/loam_wrapper.h"
#include "loam/transformMaintenance.h"
#include "loam/laserMapping.h"
#include "loam/scanRegistration.h"
#include "loam/laserOdometry.h"

struct data
{
    nav_msgs::OdometryPtr * ptr_od;
    sensor_msgs::PointCloud2Ptr * ptr_pc;
    sensor_msgs::ImagePtr * ptr_img;
};

int main( int argc, char** argv )
{
    ros::init(argc, argv, "LOAM");

    // get data from rosbag file
    rosbag::Bag bag;
    bag.open("/home/sebastian/Dropbox/aria/Zhang_data_for_loam/robot_city_bridge/robot_city_bridge.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("/sync_scan_cloud_filtered"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // extract data from bag file
    std::vector<sensor_msgs::PointCloud2Ptr> vec_lidar;
    ros::NodeHandle nh;
    ros::Publisher pubLaserOdometry2 = nh.advertise<nav_msgs::Odometry> ("/cam_to_init_2", 5);
    ros::Publisher pubOdomBefMapped = nh.advertise<nav_msgs::Odometry> ("/bef_mapped_to_init_2", 5);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init_2", 5);
    ros::Publisher pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 1);
    ros::Publisher pubLaserCloudExtreCur = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_extre_cur", 2);
    ros::Publisher pubLaserCloudLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_last", 2);
    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry> ("/cam_to_init", 5);
    ros::Publisher pubLaserCloudLast2 = nh.advertise<sensor_msgs::PointCloud2> ("/laser_cloud_last_2", 2);

    // Transform Maintance
    tf::TransformBroadcaster tfBroadcaster2;
    transformMaintenance::transformMaintenance transformMain(&pubLaserOdometry2,&tfBroadcaster2);

    // Laser Mapping
    laserMapping::laserMapping laserMap(&pubOdomBefMapped,&pubOdomAftMapped,&pubLaserCloudSurround);

    // Scan Registration
    scanRegistration::scanRegistration scanReg(&pubLaserCloudExtreCur,&pubLaserCloudLast);

    // Laser Odometry
    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform laserOdometryTrans;
    laserOdometry::laserOdometry laserOd(&tfBroadcaster,&laserOdometryTrans,&pubLaserOdometry,&pubLaserCloudLast2);

    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::PointCloud2Ptr pc = m.instantiate<sensor_msgs::PointCloud2>();
        if (pc != NULL)
        {

            // Registration
            sensor_msgs::PointCloud2 outExtreCur2;
            sensor_msgs::PointCloud2 outCloudLast2;
            scanReg.laserCloudHandler(pc,outExtreCur2,outCloudLast2);
            if(outCloudLast2.width>0)
            {
                laserOd.laserCloudLastHandler(outCloudLast2);
                outCloudLast2.fields[3].name = "intensity";
            }
            if(outExtreCur2.width>0)
            {
                laserOd.laserCloudExtreCurHandler(outExtreCur2);
            }


            // Laser Odometry
            sensor_msgs::PointCloud2 pub;
            nav_msgs::Odometry pubOdo;
            laserOd.main_laserOdometry(pub, pubOdo);


            // Lasser Mapping
            laserMap.laserOdometryHandler(pubOdo);

            if (pub.width>0)
                laserMap.laserCloudLastHandler(pub);



            sensor_msgs::PointCloud2 laser_cloud_surround;
            nav_msgs::Odometry odomBefMapped;
            nav_msgs::Odometry odomAftMapped;
            laserMap.loop(laser_cloud_surround, odomBefMapped, odomAftMapped);

            // maintanance
            nav_msgs::Odometry outlaserOdometry2;

            transformMain.odomAftMappedHandler(odomAftMapped);
            transformMain.odomBefMappedHandler(odomBefMapped);
            transformMain.laserOdometryHandler(pubOdo,outlaserOdometry2);

        }
    }

    bag.close();
    return 0;
}
