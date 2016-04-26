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
    int maxId = 130;
    KITTI kitti(0,maxId,offset);
    loam_wrapper loam;



    std::vector<Eigen::Matrix4d> T_result;
    T_result.push_back(Eigen::Matrix4d::Identity());
    std::vector<Eigen::Matrix4d> T_result_delta;

    Eigen::Matrix4d T_offset = kitti.getGroundTruthByID(offset);
    Eigen::Matrix4d T_all_od = kitti.getGroundTruthByID(offset);
    Eigen::Matrix4d T_all_map = kitti.getGroundTruthByID(offset);
    Eigen::Matrix4d T_diff_od_map;

    Eigen::Matrix4d T = kitti.getVelo_to_cam_T();

    for (int i=offset;i<maxId-1;i++)
    {
        // velo1
        pcl::PointCloud<pcl::PointXYZ>::Ptr velo_1(new pcl::PointCloud<pcl::PointXYZ>());
        kitti.getOneVel(velo_1,i);
        pcl::transformPointCloud (*velo_1, *velo_1, T);

        // velo2
        pcl::PointCloud<pcl::PointXYZ>::Ptr velo_2(new pcl::PointCloud<pcl::PointXYZ>());
        kitti.getOneVel(velo_2,i+1);
        pcl::transformPointCloud (*velo_2, *velo_2, T);

        // transform according to ground truth
        Eigen::Matrix4d T_1 = kitti.getGroundTruthByID(i);
        Eigen::Matrix4d T_2 = kitti.getGroundTruthByID(i+1);
        Eigen::Matrix4d T_diff = T_1.inverse() * T_2;
        pcl::transformPointCloud (*velo_2, *velo_2, T_diff);

        pcl::PointCloud<pcl::PointXYZ>::Ptr velo_1_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter;
        downSizeFilter.setInputCloud(velo_1);
        downSizeFilter.setLeafSize(0.1f, 0.1f, 0.1f);
        downSizeFilter.filter(*velo_1_filtered);

        pcl::PointCloud<pcl::PointXYZ>::Ptr velo_2_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter2;
        downSizeFilter2.setInputCloud(velo_2);
        downSizeFilter2.setLeafSize(0.1f, 0.1f, 0.1f);
        downSizeFilter2.filter(*velo_2_filtered);





        // icp
        ICP icp;
        icp.setSurfaceMap(velo_1_filtered);
        icp.setInputCloud(velo_2_filtered);
        icp.testGetPoint2PlaneDistance();
        icp.testGetPlaneJacobi();
        icp.doICP();

        Eigen::Matrix4d T_back = icp.getBestResult();







        loam.publishFirst(velo_1_filtered);



        pcl::transformPointCloud (*velo_2_filtered, *velo_2_filtered, T_back);


        loam.publishSecond(velo_2_filtered);


        icp.reset();
        Eigen::Matrix4d T_gt = kitti.getGroundTruthDeltaByID(i+1);


        //    T_result_delta.push_back(T_back);

        //    T_all_od = T_offset*loam.T_total_od;

        T_back = T_diff * T_back;
        T_all_map = T_all_map*T_back;
        //    T_diff_od_map = kitti.poseDelta(T_all_od,T_all_map);
        std::cout << "[INFO]: i=" << i << std::endl;
        std::cout << "T_gt delta" << T_gt << std::endl;
        std::cout << "T_back:" << T_back << std::endl;
        //    std::cout << "T_all_od:" << T_all_od << std::endl;
        std::cout << "T_all_map:" << T_all_map << std::endl;
        std::cout << "T_gt:" << kitti.getGroundTruthByID(i) << std::endl;
        //    std::cout << "T_diff_od_map:" << T_diff_od_map << std::endl;

        T_result.push_back(T_all_map);


    }

    kitti.plotDeltaPoses(T_result,0);
    kitti.eval(T_result);
    //    kitti.writeResult(T_result_delta);




    return 0;
}



