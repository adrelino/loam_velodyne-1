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
#include <Eigen/Eigenvalues>

#include "../KITTI_util/toMATLAB.h"
#include "icp.h"


namespace laserMapping
{



class laserMapping
{
public:
    laserMapping();
    ICP icp;
    toMATLAB writerToFile;
    edge_data observing_field;
    ObservingData observingData;

    const float icp_R_break = 0.01;
    const float icp_T_break = 0.01;
    const int maxIteration = 50;

    const float leafSizeCorner = 0.5f;
    const float leafSizeSurf = 0.5f;

    const int K = 5;

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

    int laserCloudValidInd[laserCloudNum];
    int laserCloudSurroundInd[laserCloudNum];

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudMap;

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudLast;
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudOri; // These are the points which are used from the input point cloud
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudMapCorres; // These are the points which are used from the input point cloud

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr coeffSel;//(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurround;//(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudArray[laserCloudNum];

    pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeCornerFromMap;//(new pcl::KdTreeFLANN<pcl::PointXYZHSV>());
    pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeSurfFromMap;//(new pcl::KdTreeFLANN<pcl::PointXYZHSV>());

    float transformSum[6] = {0};
    float transformIncre[6] = {0};
    float transformTobeMapped[6] = {0};
    float transformBefMapped[6] = {0};
    float transformAftMapped[6] = {0};

    ros::NodeHandle nh;
    ros::Publisher pubMapBeforeNewInput;
    ros::Publisher pubMapAfterNewInput;


    pcl::PointXYZHSV pointOri, pointProj, coeff;

    cv::Mat matA0;//(5, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matB0;//(5, 1, CV_32F, cv::Scalar::all(-1));
    cv::Mat matX0;//(3, 1, CV_32F, cv::Scalar::all(0));

    cv::Mat matA1;//(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matD1;//(1, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matV1;//(3, 3, CV_32F, cv::Scalar::all(0));

    bool init = false;

    void laserCloudLastHandler(const sensor_msgs::PointCloud2 &laserCloudLast2);
    void laserOdometryHandler(const nav_msgs::Odometry &laserOdometry);
    void loop();
    Eigen::Matrix4d T_transform;
    void resetMap();
private:
    void transformAssociateToMap();
    void transformUpdate();
    void pointAssociateToMap(pcl::PointXYZHSV *pi, pcl::PointXYZHSV *po);
    void processSurfPoints(pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudSurfFromMap, int iterCount, pcl::PointXYZHSV pointSel, float deltaT);
    void processCorner(pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudCornerFromMap, pcl::PointXYZHSV &searchPoint, pcl::PointXYZHSV &pointOriginal);
    void processCorner(pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudCornerFromMap, pcl::PointXYZHSV pointSel);
    void setTransformationMatrix(double rx, double ry, double rz, double tx, double ty, double tz);
    void doICP(pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudSurfFromMap, pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudCornerFromMap);
    void doICP_new(pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudSurfFromMap, pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudCornerFromMap);
    void associate(pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudSurfFromMap, int iterCount, float deltaT);
    void solveCV();
    void solveEigen(float &deltaR, float &deltaT);
    void pointAssociateToMapEig(pcl::PointXYZHSV *pi, pcl::PointXYZHSV *po);
    void pointAssociateToMapEigInv(pcl::PointXYZHSV *pi, pcl::PointXYZHSV *po);
    void extractFeatures(pcl::PointCloud<pcl::PointXYZHSV>::Ptr corners, pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfs, pcl::PointCloud<pcl::PointXYZHSV>::Ptr input);
    void createLaserCloudSurround(int laserCloudSurroundNum);
    void downSampleCloud(pcl::PointCloud<pcl::PointXYZHSV>::Ptr inPC, pcl::PointCloud<pcl::PointXYZHSV>::Ptr outPC, float leafSize);
    void storeMapInCubes(pcl::PointCloud<pcl::PointXYZHSV>::Ptr input);
    void downsampleLaserCloudArray(int laserCloudValidNum);
    void downsampleInputCloud();
    void saveMap();
};
}
