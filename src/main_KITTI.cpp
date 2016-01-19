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
    int offset = 0;
    KITTI kitti(0,150,offset);
    loam_wrapper loam;




    //    std::vector<Eigen::Matrix3d> Rs;
    //    std::vector<Eigen::Vector3d> ts;
    //    kitti.getGtCameraPoses(Rs,ts);
    //    pcl::PointCloud<pcl::PointXYZ> msg;
    //    kitti.getPointCloud(msg,0);
    //    pcl::PointCloud<pcl::PointXYZ> msg2;
    //    kitti.getPointCloud(msg2,2);
    //    Eigen::Matrix3d velo_to_cam_R = Rs[2];
    //    Eigen::Vector3d velo_to_cam_t = ts[2];
    //    Eigen::Matrix4d T;
    //    T << velo_to_cam_R(0,0), velo_to_cam_R(0,1), velo_to_cam_R(0,2), velo_to_cam_t(0),
    //            velo_to_cam_R(1,0), velo_to_cam_R(1,1), velo_to_cam_R(1,2), velo_to_cam_t(1),
    //            velo_to_cam_R(2,0), velo_to_cam_R(2,1), velo_to_cam_R(2,2), velo_to_cam_t(2),
    //            0 , 0 , 0 ,1;
    //    std::cout << "T=" << T << std::endl;
    //    pcl::transformPointCloud (msg2, msg2, T);
    //    sensor_msgs::PointCloud2 first;
    //    pcl::toROSMsg(msg,first);
    //    sensor_msgs::PointCloud2 second;
    //    pcl::toROSMsg(msg2,second);
    //    sleep(1);
    //    first.header.stamp = ros::Time().now();
    //    first.header.frame_id = "/camera_init_2";
    //    loam.publishInput(first);
    //    sleep(1);
    //    second.header.stamp = ros::Time().now();
    //    second.header.frame_id = "/camera_init_2";
    //    loam.publishLaserCloudLast2(second);






    //    loam.mappingTest(first,second,T);
    //    std::cout << "---------" << std::endl;


    //    kitti.getPointCloud(msg,1);
    //    kitti.getPointCloud(msg2,2);
    //    //pcl::transformPointCloud (msg, msg, T);
    //    pcl::toROSMsg(msg,first);
    //    velo_to_cam_R = Rs[2];
    //    velo_to_cam_t = ts[2];
    //    Eigen::Matrix4d T2;
    //    T2 << velo_to_cam_R(0,0), velo_to_cam_R(0,1), velo_to_cam_R(0,2), velo_to_cam_t(0),
    //            velo_to_cam_R(1,0), velo_to_cam_R(1,1), velo_to_cam_R(1,2), velo_to_cam_t(1),
    //            velo_to_cam_R(2,0), velo_to_cam_R(2,1), velo_to_cam_R(2,2), velo_to_cam_t(2),
    //            0 , 0 , 0 ,1;
    //    T2 = T.inverse() * T2;
    //    std::cout << "T2=" << T2 << std::endl;

    //    pcl::toROSMsg(msg2,second);
    //    first.header.stamp = ros::Time().now();
    //    first.header.frame_id = "/camera_init_2";
    //    //loam.publishInput(first);
    //    sleep(1);
    //    second.header.stamp = ros::Time().now();
    //    second.header.frame_id = "/camera_init_2";
    //    //loam.publishLaserCloudLast2(second);
    //    loam.mappingTest(first,second,T2);

    //    return 0;










    std::vector<Eigen::Matrix4d> T_result;
    std::vector<Eigen::Matrix4d> T_result_delta;
    T_result.push_back(Eigen::Matrix4d::Identity());
    Eigen::Matrix4d T_all_od = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_all_map = Eigen::Matrix4d::Identity();
    for (int i=offset; i<kitti.size()-1;i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
        kitti.getOneVel(laserCloudIn,i);
       // kitti.dispLidarInImage(laserCloudIn,i);
        Eigen::Matrix4d T = kitti.getVelo_to_cam_T();
        pcl::transformPointCloud (*laserCloudIn, *laserCloudIn, T);

        //loam.publishGT(gt[i+1]);

        Eigen::Matrix4d T_back= loam.newInPCKITTI(laserCloudIn);
        T_result_delta.push_back(T_back);

        T_all_od = loam.T_total_od;
        T_all_map = loam.T_total_map;
        std::cout << "[INFO]: i=" << i << std::endl;
        std::cout << "T_back:" << T_back << std::endl;
        std::cout << "T_all_od:" << T_all_od << std::endl;
        std::cout << "T_all_map:" << T_all_map << std::endl;

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



