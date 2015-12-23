#include "loam_wrapper.h"

loam_wrapper::loam_wrapper()
{
    pubLaserOdometry2 = nh.advertise<nav_msgs::Odometry> ("/cam_to_init_2", 5);
    pubOdomBefMapped = nh.advertise<nav_msgs::Odometry> ("/bef_mapped_to_init_2", 5);
    pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init_2", 5);
    pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 1);
    pubLaserCloudExtreCur = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_extre_cur", 2);
    pubLaserCloudLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_last", 2);
    pubLaserOdometry = nh.advertise<nav_msgs::Odometry> ("/cam_to_init", 5);
    pubLaserCloudLast2 = nh.advertise<sensor_msgs::PointCloud2> ("/laser_cloud_last_2", 2);
    pubLaserInput = nh.advertise<sensor_msgs::PointCloud2> ("/input", 5);

    transformMain = new transformMaintenance::transformMaintenance(&pubLaserOdometry2,&tfBroadcaster2);
    laserMap = new laserMapping::laserMapping(&pubOdomBefMapped,&pubOdomAftMapped,&pubLaserCloudSurround);
    scanReg = new scanRegistration::scanRegistration(&pubLaserCloudExtreCur,&pubLaserCloudLast);
    laserOd = new laserOdometry::laserOdometry(&tfBroadcaster,&laserOdometryTrans,&pubLaserOdometry,&pubLaserCloudLast2);
}

void loam_wrapper::newInPC(sensor_msgs::PointCloud2Ptr pc)
{
    // Registration
    scanReg->laserCloudHandler(pc,outExtreCur2,outCloudLast2);
    if(outCloudLast2.width>0)
    {
        laserOd->laserCloudLastHandler(outCloudLast2);
        outCloudLast2.fields[3].name = "intensity";
    }
    if(outExtreCur2.width>0)
    {
        laserOd->laserCloudExtreCurHandler(outExtreCur2);
    }


    // Laser Odometry
    laserOd->main_laserOdometry(pub, pubOdo);

    // Lasser Mapping
    laserMap->laserOdometryHandler(pubOdo);

    if (pub.width>0)
        laserMap->laserCloudLastHandler(pub);

    laserMap->loop(laser_cloud_surround, odomBefMapped, odomAftMapped);

    // maintanance
    transformMain->odomAftMappedHandler(odomAftMapped);
    transformMain->odomBefMappedHandler(odomBefMapped);
    transformMain->laserOdometryHandler(pubOdo,outlaserOdometry2);

}
