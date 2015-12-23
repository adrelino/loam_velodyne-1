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

#include "util/KITTI.h"

struct data
{
    nav_msgs::OdometryPtr * ptr_od;
    sensor_msgs::PointCloud2Ptr * ptr_pc;
    sensor_msgs::ImagePtr * ptr_img;
};

int main( int argc, char** argv )
{
    ros::init(argc, argv, "KITTI");


    KITTI kitti;

    loam_wrapper loam;

    sensor_msgs::PointCloud2 pc;
    for (int i=0; i<kitti.velpoints.size();i++)
    {
        kitti.getPointCloud2(pc);
        sleep(4);

        loam.publishInput(pc);

       // sensor_msgs::PointCloud2Ptr pc_ptr(&pc);

       // loam.newInPC(pc_ptr);

    }


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
