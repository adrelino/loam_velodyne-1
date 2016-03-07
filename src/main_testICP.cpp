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
//#include "cv_bridge/cv_bridge.h"
#define foreach BOOST_FOREACH

#include "loam/loam_wrapper.h"
#include "loam/transformMaintenance.h"
#include "loam/laserMapping.h"
#include "loam/scanRegistration.h"
#include "loam/laserOdometry.h"

#include "KITTI_util/KITTI.h"

#include "KITTI_util/toMATLAB.h"
#include "loam/icp.h"

int main( int argc, char** argv )
{
    ros::init(argc, argv, "KITTI");
    int offset = 0;
    int maxId = 1000;
    KITTI kitti(0,maxId,offset);
    loam_wrapper loam;



    //std::vector<Eigen::Matrix4d> T_result;
    std::vector<Eigen::Matrix4d> T_result_delta;

    Eigen::Matrix4d T_offset = kitti.getGroundTruthByID(offset);
    Eigen::Matrix4d T_all_od = kitti.getGroundTruthByID(offset);
    Eigen::Matrix4d T_all_map = kitti.getGroundTruthByID(offset);
    Eigen::Matrix4d T_diff_od_map;

    Eigen::Matrix4d T = kitti.getVelo_to_cam_T();

    // velo1
    pcl::PointCloud<pcl::PointXYZ>::Ptr velo_1(new pcl::PointCloud<pcl::PointXYZ>());
    kitti.getOneVel(velo_1,0);
    pcl::transformPointCloud (*velo_1, *velo_1, T);

    // velo2
    pcl::PointCloud<pcl::PointXYZ>::Ptr velo_2(new pcl::PointCloud<pcl::PointXYZ>());
    kitti.getOneVel(velo_2,1);
    pcl::transformPointCloud (*velo_2, *velo_2, T);

    // transform according to ground truth
    Eigen::Matrix4d T_1 = kitti.getGroundTruthByID(1);
    Eigen::Matrix4d T_2 = kitti.getGroundTruthByID(2);
    Eigen::Matrix4d T_diff = T_1.inverse() * T_2;
    //pcl::transformPointCloud (*velo_2, *velo_2, T_diff);

    //
//    ros::NodeHandle nh;
//    ros::Publisher pub_velo_1 = nh.advertise<sensor_msgs::PointCloud2>("/velo_1", 10);
//    ros::Publisher pub_velo_2 = nh.advertise<sensor_msgs::PointCloud2>("/velo_2", 10);
//    sensor_msgs::PointCloud2 pc_velo_1;
//    pcl::toROSMsg(*velo_1, pc_velo_1);
//    pc_velo_1.header.stamp = ros::Time().now();
//    pc_velo_1.header.frame_id = "/world";
//    pub_velo_1.publish(pc_velo_1);
//        pub_velo_1.publish(pc_velo_1);

//    sensor_msgs::PointCloud2 pc_velo_2;
//    pcl::toROSMsg(*velo_2, pc_velo_2);
//    pc_velo_2.header.stamp = ros::Time().now();
//    pc_velo_2.header.frame_id = "/world";
//    pub_velo_2.publish(pc_velo_2);
//     pub_velo_2.publish(pc_velo_2);

    // icp
    ICP icp;
    //icp.testCovariance();
    //icp.testLVM();
    icp.setSurfaceMap(velo_1);
    icp.setInputCloud(velo_2);
    icp.doICP();
    Eigen::Matrix4f T_result = icp.getTransformationMatrix();
    pcl::transformPointCloud (*velo_2, *velo_2, T_result);

    loam.publishFirst(velo_1);
  loam.publishSecond(velo_2);

  loam.publishFirst(velo_1);
loam.publishSecond(velo_2);
loam.publishFirst(velo_1);
loam.publishSecond(velo_2);
loam.publishFirst(velo_1);
loam.publishSecond(velo_2);
loam.publishFirst(velo_1);
loam.publishSecond(velo_2);
loam.publishFirst(velo_1);
loam.publishSecond(velo_2);
loam.publishFirst(velo_1);
loam.publishSecond(velo_2);
loam.publishFirst(velo_1);
loam.publishSecond(velo_2);

//    Eigen::Matrix4d T_gt = kitti.getGroundTruthDeltaByID(i);


//    T_result_delta.push_back(T_back);

//    T_all_od = T_offset*loam.T_total_od;
//    T_all_map = T_offset*loam.T_total_map;
//    T_diff_od_map = kitti.poseDelta(T_all_od,T_all_map);
//    std::cout << "[INFO]: i=" << i << std::endl;
//    std::cout << "T_back:" << T_back << std::endl;
//    std::cout << "T_all_od:" << T_all_od << std::endl;
//    std::cout << "T_all_map:" << T_all_map << std::endl;
//    std::cout << "T_gt:" << kitti.getGroundTruthByID(i) << std::endl;
//    std::cout << "T_diff_od_map:" << T_diff_od_map << std::endl;

//    T_result.push_back(T_all_map);




//    kitti.plotDeltaPoses(T_result,0);
//    kitti.eval(T_result);
//    kitti.writeResult(T_result_delta);




    return 0;
}



