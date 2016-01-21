#pragma once

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Geometry>


namespace laserMapping
{

class laserMapping
{
public:
    const int maxIteration = 50;
    laserMapping(ros::Publisher * pubOdomBefMapped, ros::Publisher * pubOdomAftMapped, ros::Publisher * pubLaserCloudSurround);
    void laserCloudLastHandlerVelo(const pcl::PointCloud<pcl::PointXYZHSV>::Ptr inLaserCloudLast);
    void laserOdometryHandlerVelo(const Eigen::Matrix4d T);

    const double PI = 3.1415926;
    const double rad2deg = 180 / PI;
    const double deg2rad = PI / 180;

    double timeLaserCloudLast;
    double timeLaserOdometry;

    bool newLaserCloudLast = false;
    bool newLaserOdometry = false;

    const int laserCloudCenWidth = 1;
    const int laserCloudCenHeight = 1;
    const int laserCloudCenDepth = 1;
    static const int laserCloudWidth = 15;
    static const int laserCloudHeight = 15;
    static const int laserCloudDepth = 15;
    static const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;

    int laserCloudValidInd[27];
    int laserCloudSurroundInd[27];

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudCorner;
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudSurf;
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudCorner2;
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudSurf2;
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudLast;
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudOri;

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr coeffSel;//(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudFromMap;//(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudCornerFromMap;//(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudSurfFromMap;//(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurround;//(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudArray[laserCloudNum];

    pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeCornerFromMap;//(new pcl::KdTreeFLANN<pcl::PointXYZHSV>());
    pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeSurfFromMap;//(new pcl::KdTreeFLANN<pcl::PointXYZHSV>());

    float transformSum[6] = {0};
    float transformIncre[6] = {0};
    float transformTobeMapped[6] = {0};
    float transformBefMapped[6] = {0};
    float transformAftMapped[6] = {0};

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    pcl::PointXYZHSV pointOri, pointSel, pointProj, coeff;

    cv::Mat matA0;//(5, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matB0;//(5, 1, CV_32F, cv::Scalar::all(-1));
    cv::Mat matX0;//(3, 1, CV_32F, cv::Scalar::all(0));

    cv::Mat matA1;//(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matD1;//(1, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matV1;//(3, 3, CV_32F, cv::Scalar::all(0));

    bool init = false;

    ros::Publisher * pubOdomBefMapped;
    ros::Publisher * pubOdomAftMapped;
    ros::Publisher * pubLaserCloudSurround;

    void laserCloudLastHandler(const sensor_msgs::PointCloud2 &laserCloudLast2);
    void laserOdometryHandler(const nav_msgs::Odometry &laserOdometry);
    void loop(sensor_msgs::PointCloud2 &laser_cloud_surround, nav_msgs::Odometry & odomBefMapped,nav_msgs::Odometry & odomAftMapped);
    Eigen::Matrix4d T_transform;
private:
    void transformAssociateToMap();
    void transformUpdate();
    void pointAssociateToMap(pcl::PointXYZHSV *pi, pcl::PointXYZHSV *po);
    void processSurfPoints(int iterCount);
    void processCorner();
    void setTransformationMatrix(double rx, double ry, double rz, double tx, double ty, double tz);
    void doICP();
    void associate(int iterCount);
    void solveCV();
    void solveEigen(float &deltaR, float &deltaT);
    void pointAssociateToMapEig(pcl::PointXYZHSV *pi, pcl::PointXYZHSV *po);
    void extractFeatures();
    void createLaserCloudSurround(int laserCloudSurroundNum);
};
}
