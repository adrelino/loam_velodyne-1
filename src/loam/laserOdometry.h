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

#define SQRDIST(x1,y1,z1,x2,y2,z2) (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2)


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

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr outLaserCloudLast2;

    float transform[6] = {0};
    float transformRec[6] = {0};
    float transformSum[6] = {0};

    Eigen::Matrix4d T_transform;

    float imuRollStartCur, imuPitchStartCur, imuYawStartCur;
    float imuRollCur, imuPitchCur, imuYawCur;
    float imuShiftFromStartXCur, imuShiftFromStartYCur, imuShiftFromStartZCur;
    float imuVeloFromStartXCur, imuVeloFromStartYCur, imuVeloFromStartZCur;

    float imuRollStartLast, imuPitchStartLast, imuYawStartLast;
    float imuRollLast, imuPitchLast, imuYawLast;
    float imuShiftFromStartXLast, imuShiftFromStartYLast, imuShiftFromStartZLast;
    float imuVeloFromStartXLast, imuVeloFromStartYLast, imuVeloFromStartZLast;

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
    void laserCloudLastHandlerVelo(const pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsSharp, const pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsLessSharp, const pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsFlat, const pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlatDS);
    void laserCloudExtreCurHandlerVelo(const pcl::PointCloud<pcl::PointXYZHSV>::Ptr inLaserCloudExtreCur3);
    void setTransformationMatrix(double rx, double ry, double rz, double tx, double ty, double tz);
private:
    int processEdgePoint(pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeSurfPtr, pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudSurfPtr, bool isPointSel, int laserCloudSurfNum, int i, int iterCount, int iterNum);
    int number_edge;
    int number_planar;
    int processPlanarPoint(pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeCornerPtr, pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudCornerPtr, bool isPointSel, int laserCloudCornerNum, int i);

    // for debugging
    int err_edge_pointSearchSqDis;
    int err_edge_dist;
    int err_edge_pointSelInd;
    int err_edge_minPointInd2;
    int err_planar_pointSearchSqDis;
    int err_planar_dist;
    int err_planar_pointSelInd;
};
}
