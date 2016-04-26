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

struct data
{
    nav_msgs::OdometryPtr * ptr_od;
    sensor_msgs::PointCloud2Ptr * ptr_pc;
    sensor_msgs::ImagePtr * ptr_img;
};

int main( int argc, char** argv )
{
    ros::init(argc, argv, "KITTI");
    int offset = 90;
    int maxId = 130;
    KITTI kitti(0,maxId,offset);
    loam_wrapper loam;

    toMATLAB temp;

    std::vector<Eigen::Matrix4d> T_result;
    std::vector<Eigen::Matrix4d> T_result_delta;
    //T_result.push_back(Eigen::Matrix4d::Identity());
    Eigen::Matrix4d T_offset = kitti.getGroundTruthByID(offset);
    Eigen::Matrix4d T_all_od = kitti.getGroundTruthByID(offset);
    Eigen::Matrix4d T_all_map = kitti.getGroundTruthByID(offset);
    Eigen::Matrix4d T_diff_od_map;
    for (int i=offset; i<maxId;i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
        kitti.getOneVel(laserCloudIn,i);
        std::stringstream filename;
        filename << "input_" << i << ".csv";
        temp.writePCLToCSV(filename.str(),laserCloudIn);
       // kitti.dispLidarInImage(laserCloudIn,i);
        Eigen::Matrix4d T = kitti.getVelo_to_cam_T();
        pcl::transformPointCloud (*laserCloudIn, *laserCloudIn, T);

        //loam.publishGT(gt[i+1]);
        Eigen::Matrix4d T_gt = kitti.getGroundTruthDeltaByID(i);
        //Eigen::Matrix4d T_back= loam.newInPCKITTI(laserCloudIn);
        Eigen::Matrix4d T_back= loam.mapTraining(laserCloudIn,T_gt);
        T_result_delta.push_back(T_back);

        T_all_od = T_offset*loam.T_total_od;
        T_all_map = T_offset*loam.T_total_map;
        T_diff_od_map = kitti.poseDelta(T_all_od,T_all_map);
        std::cout << "[INFO]: i=" << i << std::endl;
        std::cout << "T_back:" << T_back << std::endl;
        std::cout << "T_all_od:" << T_all_od << std::endl;
        std::cout << "T_all_map:" << T_all_map << std::endl;
        std::cout << "T_gt:" << kitti.getGroundTruthByID(i) << std::endl;
        std::cout << "T_diff_od_map:" << T_diff_od_map << std::endl;

        T_result.push_back(T_all_map);


    }

    kitti.plotDeltaPoses(T_result,0);
    kitti.eval(T_result);
    kitti.writeResult(T_result_delta);




    return 0;
}



