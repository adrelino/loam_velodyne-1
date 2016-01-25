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

#include "KITTI_util/KITTI.h"

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
        //T_diff_od_map = kitti.computeError(T_all_od,T_all_map);
        std::cout << "[INFO]: i=" << i << std::endl;
        std::cout << "T_back:" << T_back << std::endl;
        std::cout << "T_all_od:" << T_all_od << std::endl;
        std::cout << "T_all_map:" << T_all_map << std::endl;
        std::cout << "T_gt:" << kitti.getGroundTruthByID(i) << std::endl;
        //std::cout << "T_diff_od_map:" << T_diff_od_map << std::endl;

        T_result.push_back(T_all_map);

        //std::vector<veloPoint> velpoints2;
        //kitti.getOneVel(velpoints2,i);
        //pcl::PointCloud<pcl::PointXYZHSV>::Ptr temp(new pcl::PointCloud<pcl::PointXYZHSV>());
        //Eigen::Matrix4d T_in = T.inverse();
        //pcl::transformPointCloud (*(loam.cornerPointsSharpLast), *temp, T_in);
        //kitti.dispLidarInImage(temp,i);

        //sleep(5);
    }

    kitti.eval(T_result);
    kitti.writeResult(T_result_delta);


    //    Eigen::Matrix3Xd P0(3,4);
    //    P0 = getP0();
    //    Eigen::Matrix3Xd P2(3,4);
    //    P2 = getP2();
    //    Eigen::Matrix3Xd P_rect_02(3,4);
    //    P_rect_02 = getP_rect_02();
    //    Eigen::Matrix3f K=getK();



    //    int num = 15;

    //    std::vector<cv::Mat*> images_color;
    //    std::vector<std::string> files;
    //    getFiles(path_to_images_color, files);
    //    getImagesColor(files, images_color,num);

    //    std::vector<cv::Mat*> images_gray;
    //    files.clear();
    //    getFiles(path_to_images_gray, files);
    //    getImages(files, images_gray,num);



    //    std::vector<std::vector<veloPoint>> velpoints;
    //    std::vector<std::string> velo_files;
    //    getFiles(path_to_velo, velo_files);
    //    getVel(velo_files,velpoints,num);

    //    writeDepthToFile(velpoints[0]);


    //    std::vector<SE3> poses;
    //    std::vector<Eigen::Matrix3d> Rs;
    //    std::vector<Eigen::Vector3d> ts;
    //    //setCameraPoses(pathPoses,poses,Rs,ts);

    //    Eigen::Matrix4d gt_pose,gt_pose_tb;

    //    Eigen::Matrix4d gt_pose_cam,gt_pose_cam_tb;

    //    Eigen::Matrix4d T_velo_to_cam = get_T_velo_to_cam();
    //    Eigen::Matrix4d T_02 = getTrans02();
    //    Eigen::Matrix4d T_velo_02 = T_02*T_velo_to_cam;
    //    Eigen::Matrix4d imu_to_velo_T;
    //    imu_to_velo_T = get_imu_to_velo_T();



    return 0;
}



