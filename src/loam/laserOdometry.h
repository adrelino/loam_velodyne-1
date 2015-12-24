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


namespace laserOdometry
{

class laserOdometry
{
public:
    const double PI = 3.1415926;
    const double rad2deg = 180 / PI;
    const double deg2rad = PI / 180;

    bool systemInited;

    double initTime;
    double timeLasted;
    double timeLastedRec;
    double startTimeCur;
    double startTimeLast;

    double timeLaserCloudExtreCur = 0;
    double timeLaserCloudLast = 0;

    bool newLaserCloudExtreCur = false;
    bool newLaserCloudLast = false;

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreCur;//(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreLast;//(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreOri;//(new pcl::PointCloud<pcl::PointXYZHSV>());
    //pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreSel(new pcl::PointCloud<pcl::PointXYZHSV>());
    //pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreUnsel(new pcl::PointCloud<pcl::PointXYZHSV>());
    //pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreProj(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudLast;//(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudCornerLast;//(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudSurfLast;//(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudCornerLLast;//(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudSurfLLast;//(new pcl::PointCloud<pcl::PointXYZHSV>());
    //pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudSel(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr coeffSel;//(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeCornerLast;//(new pcl::KdTreeFLANN<pcl::PointXYZHSV>());
    pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeSurfLast;//(new pcl::KdTreeFLANN<pcl::PointXYZHSV>());
    pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeCornerLLast;//(new pcl::KdTreeFLANN<pcl::PointXYZHSV>());
    pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeSurfLLast;//(new pcl::KdTreeFLANN<pcl::PointXYZHSV>());

    float transform[6] = {0};
    float transformRec[6] = {0};
    float transformSum[6] = {0};

    float imuRollStartCur = 0, imuPitchStartCur = 0, imuYawStartCur = 0;
    float imuRollCur = 0, imuPitchCur = 0, imuYawCur = 0;
    float imuShiftFromStartXCur = 0, imuShiftFromStartYCur = 0, imuShiftFromStartZCur = 0;
    float imuVeloFromStartXCur = 0, imuVeloFromStartYCur = 0, imuVeloFromStartZCur = 0;

    float imuRollStartLast = 0, imuPitchStartLast = 0, imuYawStartLast = 0;
    float imuRollLast = 0, imuPitchLast = 0, imuYawLast = 0;
    float imuShiftFromStartXLast = 0, imuShiftFromStartYLast = 0, imuShiftFromStartZLast = 0;
    float imuVeloFromStartXLast = 0, imuVeloFromStartYLast = 0, imuVeloFromStartZLast = 0;

    bool imuInited = false;

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    std::vector<int> pointSelInd;

    pcl::PointXYZHSV extreOri, extreSel, extreProj, tripod1, tripod2, tripod3, coeff;

    tf::TransformBroadcaster * tfBroadcaster;
    tf::StampedTransform * laserOdometryTrans;
    ros::Publisher * pubLaserOdometry;
    ros::Publisher * pubLaserCloudLast2;

    laserOdometry(tf::TransformBroadcaster * tfBroadcaster,tf::StampedTransform * laserOdometryTrans, ros::Publisher * pubLaserOdometry, ros::Publisher * pubLaserCloudLast2);
    void TransformReset();
    void TransformToStart(pcl::PointXYZHSV *pi, pcl::PointXYZHSV *po, double startTime, double endTime);
    void TransformToEnd(pcl::PointXYZHSV *pi, pcl::PointXYZHSV *po, double startTime, double endTime);
    void AccumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz,
                            float &ox, float &oy, float &oz);
    void PluginIMURotation(float bcx, float bcy, float bcz, float blx, float bly, float blz,
                           float alx, float aly, float alz, float &acx, float &acy, float &acz);
    void laserCloudExtreCurHandler(const sensor_msgs::PointCloud2& laserCloudExtreCur2);
    void laserCloudLastHandler(const sensor_msgs::PointCloud2 &laserCloudLast2);
    void main_laserOdometry(sensor_msgs::PointCloud2 &pub, nav_msgs::Odometry &pubOdo);
};
}
