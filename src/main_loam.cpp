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

    loam_wrapper loam;

    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::PointCloud2Ptr pc = m.instantiate<sensor_msgs::PointCloud2>();
        if (pc != NULL)
        {
             //pubLaserInput.publish(pc);

            loam.newInPC(pc);

        }
    }

    bag.close();
    return 0;
}
