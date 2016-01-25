#include "loam_wrapper.h"

loam_wrapper::loam_wrapper()
{
    pubLaserOdometry2 = nh.advertise<nav_msgs::Odometry> ("/cam_to_init_2", 5);
    pubOdomBefMapped = nh.advertise<nav_msgs::Odometry> ("/bef_mapped_to_init_2", 5);
    pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init_2", 5);
    pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 1);
    pubLaserCloudExtreCur = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_extre_cur", 2);
    pubFirst = nh.advertise<sensor_msgs::PointCloud2>("/first", 1);
    pubSecond = nh.advertise<sensor_msgs::PointCloud2>("/second", 2);
    pubMap = nh.advertise<sensor_msgs::PointCloud2>("/map", 2);
    pubLaserCloudLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_last", 2);
    pubLaserOdometry = nh.advertise<nav_msgs::Odometry> ("/cam_to_init", 5);
    pubLaserCloudLast2 = nh.advertise<sensor_msgs::PointCloud2> ("/laser_cloud_last_2", 2);
    pubLaserInput = nh.advertise<sensor_msgs::PointCloud2> ("/input", 5);
    pubGT = nh.advertise<nav_msgs::Odometry> ("/ground_truth", 5);

    transformMain = new transformMaintenance::transformMaintenance(&pubLaserOdometry2,&tfBroadcaster2);
    laserMap = new laserMapping::laserMapping(&pubOdomBefMapped,&pubOdomAftMapped,&pubLaserCloudSurround);
    scanReg = new scanRegistration::scanRegistration(&pubLaserCloudExtreCur,&pubLaserCloudLast);
    scanReg2 = new scanRegistration::scanRegistration(&pubLaserCloudExtreCur,&pubLaserCloudLast);
    laserOd = new laserOdometry::laserOdometry(&tfBroadcaster,&laserOdometryTrans,&pubLaserOdometry,&pubLaserCloudLast2);

    T_total_od = Eigen::Matrix4d::Identity();
    T_total_map = Eigen::Matrix4d::Identity();
    T_gt = Eigen::Matrix4d::Identity();
    isInitialized = false;


    cornerPointsSharpLast.reset(new pcl::PointCloud<pcl::PointXYZHSV>());
    surfPointsFlatLast.reset(new pcl::PointCloud<pcl::PointXYZHSV>());
    cornerPointsLessSharpLast.reset(new pcl::PointCloud<pcl::PointXYZHSV>());
    surfPointsLessFlatDSLast.reset(new pcl::PointCloud<pcl::PointXYZHSV>());
}

loam_wrapper::~loam_wrapper()
{
    delete transformMain;
    delete laserMap;
    delete scanReg;
    delete scanReg2;
    delete laserOd;

}

void loam_wrapper::publishGT(nav_msgs::Odometry &od)
{
    gt = od;
    pubGT.publish(od);
    std::cout << "od" << od.pose.pose.position << std::endl;
}

void loam_wrapper::publishInput(sensor_msgs::PointCloud2 &pc)
{
    pubLaserInput.publish(pc);
}

void loam_wrapper::publishInput(pcl::PointCloud<pcl::PointXYZHSV>::Ptr pc)
{
    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(*pc, pc2);
    pc2.header.stamp = ros::Time().now();
    pc2.header.frame_id = "/camera_init_2";
    publishInput(pc2);
}

void loam_wrapper::publishFirst(pcl::PointCloud<pcl::PointXYZHSV>::Ptr pc)
{
    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(*pc, pc2);
    pc2.header.stamp = ros::Time().now();
    pc2.header.frame_id = "/world";
    pubFirst.publish(pc2);
}

void loam_wrapper::publishMap(pcl::PointCloud<pcl::PointXYZI>::Ptr pc)
{
    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(*pc, pc2);
    pc2.header.stamp = ros::Time().now();
    pc2.header.frame_id = "/world";
    pubMap.publish(pc2);
}

void loam_wrapper::publishSecond(pcl::PointCloud<pcl::PointXYZHSV>::Ptr pc)
{
    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(*pc, pc2);
    pc2.header.stamp = ros::Time().now();
    pc2.header.frame_id = "/world";
    pubSecond.publish(pc2);
}

void loam_wrapper::publishLaserCloudLast2(sensor_msgs::PointCloud2 &pc)
{
    pubLaserCloudLast2.publish(pc);
}

void loam_wrapper::publishLaserCloudLast2(pcl::PointCloud<pcl::PointXYZHSV>::Ptr pc)
{
    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(*pc, pc2);
    pc2.header.stamp = ros::Time().now();
    pc2.header.frame_id = "/camera_init_2";
    publishLaserCloudLast2(pc2);
}

void loam_wrapper::publishLaserCloudLast(sensor_msgs::PointCloud2 &pc)
{
    pubLaserCloudLast.publish(pc);
}

void loam_wrapper::newInPC(sensor_msgs::PointCloud2Ptr pc)
{
    // Registration
    scanReg->laserCloudHandler(pc);
    if(scanReg->laserCloudLast2.width>0)
    {
        laserOd->laserCloudLastHandler(scanReg->laserCloudLast2);
        //outCloudLast2.fields[3].name = "intensity";
    }
    if(scanReg->laserCloudExtreCur2.width>0)
    {
        laserOd->laserCloudExtreCurHandler(scanReg->laserCloudExtreCur2);
    }


    // Laser Odometry
    laserOd->main_laserOdometry(pub, pubOdo);

    // Lasser Mapping
    laserMap->laserOdometryHandler(pubOdo);

    if (pub.width>0)
        laserMap->laserCloudLastHandler(pub);

    if (pub.width>0)
        laserMap->loop(laser_cloud_surround, odomBefMapped, odomAftMapped);

    // maintanance
    transformMain->odomAftMappedHandler(odomAftMapped);
    transformMain->odomBefMappedHandler(odomBefMapped);
    transformMain->laserOdometryHandler(pubOdo,outlaserOdometry2);

}

Eigen::Matrix4d loam_wrapper::newInPCKITTI(const pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn)
{
    /// Registration
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsFlat(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlatDS(new pcl::PointCloud<pcl::PointXYZHSV>());
    scanReg->laserCloudHandlerVelo(laserCloudIn,cornerPointsSharp,cornerPointsLessSharp,surfPointsFlat,surfPointsLessFlatDS,false,false);

    Eigen::Matrix4d T_od = Eigen::Matrix4d::Identity();

    if (isInitialized)
    {


        //publishFirst(surfPointsLessFlatDSLast);
        /// Odometry
        laserOd->laserCloudLastHandlerVelo(cornerPointsSharpLast,cornerPointsLessSharpLast,surfPointsFlatLast,surfPointsLessFlatDSLast);


        pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreCur(new pcl::PointCloud<pcl::PointXYZHSV>());
        *laserCloudExtreCur += *cornerPointsSharp;
        *laserCloudExtreCur += *surfPointsFlat;

        laserOd->laserCloudExtreCurHandlerVelo(laserCloudExtreCur);

        //publishSecond(laserCloudExtreCur);

        laserOd->main_laserOdometry(pub, pubOdo);
        T_od = laserOd->T_transform;
        pcl::transformPointCloud (*(laserOd->outLaserCloudLast2), *(laserOd->outLaserCloudLast2), T_od);
        T_od = T_od.inverse();
        //        pcl::transformPointCloud (*laserCloudExtreCur, *laserCloudExtreCur, T);
        //        publishSecond(laserCloudExtreCur);



        T_total_od = T_total_od * T_od;



        /// Lasser Mapping

        laserMap->laserOdometryHandlerVelo(T_total_od);


        laserMap->laserCloudLastHandlerVelo(laserOd->outLaserCloudLast2);


        laserMap->loop(laser_cloud_surround, odomBefMapped, odomAftMapped);
        T_total_map = laserMap->T_transform;
        //publishMap(laserMap->laserCloudSurround);



    }
    isInitialized = true;
    *cornerPointsSharpLast = *cornerPointsSharp;
    *surfPointsFlatLast = *surfPointsFlat;
    *cornerPointsLessSharpLast = *cornerPointsLessSharp;
    *surfPointsLessFlatDSLast = *surfPointsLessFlatDS;

    return T_od;

}

Eigen::Matrix4d loam_wrapper::mapTraining(const pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn, const Eigen::Matrix4d T_gt)
{
    /// Registration
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsFlat(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlatDS(new pcl::PointCloud<pcl::PointXYZHSV>());
    scanReg->laserCloudHandlerVelo(laserCloudIn,cornerPointsSharp,cornerPointsLessSharp,surfPointsFlat,surfPointsLessFlatDS,false,false);

    Eigen::Matrix4d T_od = Eigen::Matrix4d::Identity();

    if (isInitialized)
    {



        /// Odometry
        laserOd->laserCloudLastHandlerVelo(cornerPointsSharpLast,cornerPointsLessSharpLast,surfPointsFlatLast,surfPointsLessFlatDSLast);


        pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreCur(new pcl::PointCloud<pcl::PointXYZHSV>());
        *laserCloudExtreCur += *cornerPointsSharp;
        *laserCloudExtreCur += *surfPointsFlat;

        laserOd->laserCloudExtreCurHandlerVelo(laserCloudExtreCur);

        //publishSecond(laserCloudExtreCur);
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr outLaserCloudLast2(new pcl::PointCloud<pcl::PointXYZHSV>());
        *outLaserCloudLast2 = *cornerPointsSharp;
        *outLaserCloudLast2 += *cornerPointsLessSharp;
        *outLaserCloudLast2 += *surfPointsFlat;
        *outLaserCloudLast2 += *surfPointsLessFlatDS;


        //laserOd->main_laserOdometry(pub, pubOdo);
        //T_od = laserOd->T_transform;
        T_od = T_gt.inverse();

        T_od = T_od.inverse();
        //pcl::transformPointCloud (*outLaserCloudLast2, *outLaserCloudLast2, T_od);
        pcl::transformPointCloud (*laserCloudExtreCur, *laserCloudExtreCur, T_od);

        publishFirst(surfPointsLessFlatDSLast);

        publishSecond(laserCloudExtreCur);



        T_total_od = T_total_od * T_od;



        /// Lasser Mapping
        laserMap->laserOdometryHandlerVelo(T_total_od);
        laserMap->laserCloudLastHandlerVelo(outLaserCloudLast2);
        laserMap->loop(laser_cloud_surround, odomBefMapped, odomAftMapped);
        T_total_map = laserMap->T_transform;
        //publishMap(laserMap->laserCloudSurround);



    }
    else
    {
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr outLaserCloudLast2(new pcl::PointCloud<pcl::PointXYZHSV>());
        *outLaserCloudLast2 = *cornerPointsSharpLast;
        *outLaserCloudLast2 += *cornerPointsLessSharpLast;
        *outLaserCloudLast2 += *surfPointsLessFlatDSLast;
        *outLaserCloudLast2 += *surfPointsFlatLast;
        laserMap->laserOdometryHandlerVelo(T_od);
        laserMap->laserCloudLastHandlerVelo(outLaserCloudLast2);
        laserMap->loop(laser_cloud_surround, odomBefMapped, odomAftMapped);
    }
    isInitialized = true;
    *cornerPointsSharpLast = *cornerPointsSharp;
    *surfPointsFlatLast = *surfPointsFlat;
    *cornerPointsLessSharpLast = *cornerPointsLessSharp;
    *surfPointsLessFlatDSLast = *surfPointsLessFlatDS;



    return T_od;

}

void loam_wrapper::mappingTest(sensor_msgs::PointCloud2 &pc, sensor_msgs::PointCloud2 &nextpc, Eigen::Matrix4d T)
{
    //    /// Registration
    //    sensor_msgs::PointCloud2Ptr pc_ptr2(new sensor_msgs::PointCloud2);
    //    *pc_ptr2 = pc;
    //    scanReg2->laserCloudHandlerVelo(pc_ptr2,false,true);
    //    std::cout << "---------" << std::endl;
    //    sensor_msgs::PointCloud2Ptr pc_ptr(new sensor_msgs::PointCloud2);
    //    *pc_ptr = nextpc;
    //    scanReg->laserCloudHandlerVelo(pc_ptr,true,false);
    //    std::cout << "---------" << std::endl;

    //    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZHSV>());
    //    pcl::fromROSMsg(scanReg->laserCloudExtreCur2, *laserCloudIn);
    //    pcl::transformPointCloud (*laserCloudIn, *laserCloudIn, T);
    //    sensor_msgs::PointCloud2 second;
    //    pcl::toROSMsg(*laserCloudIn,second);


    //    /// Odometry
    //    if(scanReg2->laserCloudLast2.width>0)
    //        laserOd->laserCloudLastHandler(scanReg2->laserCloudLast2);
    //    if(scanReg->laserCloudExtreCur2.width>0)
    //        laserOd->laserCloudExtreCurHandler(second);

    //    std::cout << "---------" << std::endl;
    //    laserOd->main_laserOdometry(pub, pubOdo);

    //    /// Lasser Mapping
    //    //if (pubOdo.width>0)
    //    laserMap->laserOdometryHandler(gt);

    //    if (pub.width>0)
    //        laserMap->laserCloudLastHandler(pub);

    //    if (pub.width>0)
    //        laserMap->loop(laser_cloud_surround, odomBefMapped, odomAftMapped);

    //    /// maintanance
    //    transformMain->odomAftMappedHandler(odomAftMapped);
    //    transformMain->odomBefMappedHandler(odomBefMapped);
    //    transformMain->laserOdometryHandler(pubOdo,outlaserOdometry2);

}

