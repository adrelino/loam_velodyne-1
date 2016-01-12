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
    pubGT = nh.advertise<nav_msgs::Odometry> ("/ground_truth", 5);

    transformMain = new transformMaintenance::transformMaintenance(&pubLaserOdometry2,&tfBroadcaster2);
    laserMap = new laserMapping::laserMapping(&pubOdomBefMapped,&pubOdomAftMapped,&pubLaserCloudSurround);
    scanReg = new scanRegistration::scanRegistration(&pubLaserCloudExtreCur,&pubLaserCloudLast);
    scanReg2 = new scanRegistration::scanRegistration(&pubLaserCloudExtreCur,&pubLaserCloudLast);
    laserOd = new laserOdometry::laserOdometry(&tfBroadcaster,&laserOdometryTrans,&pubLaserOdometry,&pubLaserCloudLast2);

    T_total = Eigen::Matrix4d::Identity();
    T_gt = Eigen::Matrix4d::Identity();
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

void loam_wrapper::newInPCKITTI(sensor_msgs::PointCloud2 &pc, sensor_msgs::PointCloud2 &nextpc, Eigen::Matrix4d inT_gt)
{
    Eigen::Matrix4d T_gt_delta = T_gt.inverse() * inT_gt;
    T_gt = inT_gt;

    /// Registration
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsFlat(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlatDS(new pcl::PointCloud<pcl::PointXYZHSV>());
    sensor_msgs::PointCloud2Ptr pc_ptr2(new sensor_msgs::PointCloud2);
    *pc_ptr2 = pc;
    scanReg2->laserCloudHandlerVelo(pc_ptr2,cornerPointsSharp,cornerPointsLessSharp,surfPointsFlat,surfPointsLessFlatDS,false,false);

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsSharp_next(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsLessSharp_next(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsFlat_next(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlatDS_next(new pcl::PointCloud<pcl::PointXYZHSV>());
    sensor_msgs::PointCloud2Ptr pc_ptr(new sensor_msgs::PointCloud2);
    *pc_ptr = nextpc;
    scanReg->laserCloudHandlerVelo(pc_ptr,cornerPointsSharp_next,cornerPointsLessSharp_next,surfPointsFlat_next,surfPointsLessFlatDS_next,false,false);

    /// Odometry
    if(scanReg2->outLaserCloudLast2->size()>0)
    {
        laserOd->laserCloudLastHandlerVelo(cornerPointsSharp,cornerPointsLessSharp,surfPointsFlat,surfPointsLessFlatDS);
    }
    if(scanReg->outLaserCloudExtreCur2->size()>0)
        laserOd->laserCloudExtreCurHandlerVelo(scanReg->outLaserCloudExtreCur2);



    laserOd->main_laserOdometry(pub, pubOdo);
    Eigen::Matrix4d T = laserOd->T_transform;
    std::cout << "T_gt=" << T_gt << std::endl;
    double translation_error = sqrt(T_gt_delta(0,3)*T_gt_delta(0,3)+T_gt_delta(1,3)*T_gt_delta(1,3)+T_gt_delta(2,3)*T_gt_delta(2,3));
    std::cout << "translation_error=" << translation_error << std::endl;
    std::cout << "T_od=" << T << std::endl;

    T_total = T*T_total;
    std::cout << "T_total=" << T_total << std::endl;
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr trans(new pcl::PointCloud<pcl::PointXYZHSV>());
    //    *trans = *scanReg->outLaserCloudExtreCur2;
    //    pcl::transformPointCloud(*trans, *trans, T_gt);
    //    publishInput(trans);
    //    publishLaserCloudLast2(scanReg2->outLaserCloudLast2);
    std::cout << "now estimate" << std::endl;
    //sleep(10);
    T = T.inverse();
    *trans = *scanReg->outLaserCloudExtreCur2;
    pcl::transformPointCloud(*trans, *trans, T);
    publishInput(trans);
    publishLaserCloudLast2(scanReg2->outLaserCloudLast2);


    //    /// Lasser Mapping
    //    //if (pubOdo.width>0)
    //    laserMap->laserOdometryHandler(pubOdo);

    //    if (pub.width>0)
    //        laserMap->laserCloudLastHandler(pub);

    //    if (pub.width>0)
    //        laserMap->loop(laser_cloud_surround, odomBefMapped, odomAftMapped);

    //    /// maintanance
    //    transformMain->odomAftMappedHandler(odomAftMapped);
    //    transformMain->odomBefMappedHandler(odomBefMapped);
    //    transformMain->laserOdometryHandler(pubOdo,outlaserOdometry2);

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

