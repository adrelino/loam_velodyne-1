#include "scanRegistration.h"

namespace scanRegistration
{

scanRegistration::scanRegistration(ros::Publisher * pubLaserCloudExtreCur, ros::Publisher * pubLaserCloudLast)
{
    laserCloudExtreCur.reset(new pcl::PointCloud<pcl::PointXYZHSV>());
    laserCloudLessExtreCur.reset(new pcl::PointCloud<pcl::PointXYZHSV>());

    outLaserCloudExtreCur2.reset(new pcl::PointCloud<pcl::PointXYZHSV>());
    outLaserCloudLast2.reset(new pcl::PointCloud<pcl::PointXYZHSV>());

    this->pubLaserCloudExtreCurPointer = pubLaserCloudExtreCur;
    this->pubLaserCloudLastPointer = pubLaserCloudLast;

    timeStart = 0;
    timeLasted = 0;
}

void scanRegistration::ShiftToStartIMU()
{
    float x1 = cos(imuYawStart) * imuShiftFromStartXCur - sin(imuYawStart) * imuShiftFromStartZCur;
    float y1 = imuShiftFromStartYCur;
    float z1 = sin(imuYawStart) * imuShiftFromStartXCur + cos(imuYawStart) * imuShiftFromStartZCur;

    float x2 = x1;
    float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
    float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

    imuShiftFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
    imuShiftFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
    imuShiftFromStartZCur = z2;
}

void scanRegistration::VeloToStartIMU()
{
    float x1 = cos(imuYawStart) * imuVeloFromStartXCur - sin(imuYawStart) * imuVeloFromStartZCur;
    float y1 = imuVeloFromStartYCur;
    float z1 = sin(imuYawStart) * imuVeloFromStartXCur + cos(imuYawStart) * imuVeloFromStartZCur;

    float x2 = x1;
    float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
    float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

    imuVeloFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
    imuVeloFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
    imuVeloFromStartZCur = z2;
}

void scanRegistration::TransformToStartIMU(pcl::PointXYZHSV *p)
{
    float x1 = cos(imuRollCur) * p->x - sin(imuRollCur) * p->y;
    float y1 = sin(imuRollCur) * p->x + cos(imuRollCur) * p->y;
    float z1 = p->z;

    float x2 = x1;
    float y2 = cos(imuPitchCur) * y1 - sin(imuPitchCur) * z1;
    float z2 = sin(imuPitchCur) * y1 + cos(imuPitchCur) * z1;

    float x3 = cos(imuYawCur) * x2 + sin(imuYawCur) * z2;
    float y3 = y2;
    float z3 = -sin(imuYawCur) * x2 + cos(imuYawCur) * z2;

    float x4 = cos(imuYawStart) * x3 - sin(imuYawStart) * z3;
    float y4 = y3;
    float z4 = sin(imuYawStart) * x3 + cos(imuYawStart) * z3;

    float x5 = x4;
    float y5 = cos(imuPitchStart) * y4 + sin(imuPitchStart) * z4;
    float z5 = -sin(imuPitchStart) * y4 + cos(imuPitchStart) * z4;

    p->x = cos(imuRollStart) * x5 + sin(imuRollStart) * y5 + imuShiftFromStartXCur;
    p->y = -sin(imuRollStart) * x5 + cos(imuRollStart) * y5 + imuShiftFromStartYCur;
    p->z = z5 + imuShiftFromStartZCur;
}

void scanRegistration::AccumulateIMUShift()
{
    float roll = imuRoll[imuPointerLast];
    float pitch = imuPitch[imuPointerLast];
    float yaw = imuYaw[imuPointerLast];
    float accX = imuAccX[imuPointerLast];
    float accY = imuAccY[imuPointerLast];
    float accZ = imuAccZ[imuPointerLast];

    float x1 = cos(roll) * accX - sin(roll) * accY;
    float y1 = sin(roll) * accX + cos(roll) * accY;
    float z1 = accZ;

    float x2 = x1;
    float y2 = cos(pitch) * y1 - sin(pitch) * z1;
    float z2 = sin(pitch) * y1 + cos(pitch) * z1;

    accX = cos(yaw) * x2 + sin(yaw) * z2;
    accY = y2;
    accZ = -sin(yaw) * x2 + cos(yaw) * z2;

    int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;
    double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];
    if (timeDiff < 0.1) {

        imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imuVeloX[imuPointerBack] * timeDiff
                + accX * timeDiff * timeDiff / 2;
        imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imuVeloY[imuPointerBack] * timeDiff
                + accY * timeDiff * timeDiff / 2;
        imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imuVeloZ[imuPointerBack] * timeDiff
                + accZ * timeDiff * timeDiff / 2;

        imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
        imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
        imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;
    }
}

void scanRegistration::laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn2)
{
    if (!systemInited) {
        initTime = laserCloudIn2->header.stamp.toSec();
        imuPointerFront = (imuPointerLast + 1) % imuQueLength;
        systemInited = true;
    }

    timeScanLast = timeScanCur;
    timeScanCur = laserCloudIn2->header.stamp.toSec();
    timeLasted = timeScanCur - initTime;

    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*laserCloudIn2, *laserCloudIn);

    bool newSweep = isNewSweep(laserCloudIn);

    // Take input cloud at copy it to PCL but only if inside of circle of 0.5m
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZHSV>());
    int cloudSize = createInsidePC(laserCloudIn,laserCloud);
    laserCloudIn->clear();

    // is published if lidar spinned once
    if (newSweep)
        haveSweep();

    //unknownImuStuff();


    /// Computes smoothness for each point
    computeSmoothness(laserCloud,cloudSize);

    /// Magic
    unknownFunction(laserCloud,cloudSize);

    // Compute Features
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsFlat(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZHSV>());
    compFeatures(cornerPointsSharp, cornerPointsLessSharp, surfPointsFlat, surfPointsLessFlat, laserCloud, cloudSize);

    // Filter less flat points
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlatDS(new pcl::PointCloud<pcl::PointXYZHSV>());
    pcl::VoxelGrid<pcl::PointXYZHSV> downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlat);
    downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
    downSizeFilter.filter(*surfPointsLessFlatDS);

    // copy
    *laserCloudExtreCur += *cornerPointsSharp;
    *laserCloudExtreCur += *surfPointsFlat;
    *laserCloudLessExtreCur += *cornerPointsLessSharp;
    *laserCloudLessExtreCur += *surfPointsLessFlatDS;

    // reset
    laserCloud->clear();
    cornerPointsSharp->clear();
    cornerPointsLessSharp->clear();
    surfPointsFlat->clear();
    surfPointsLessFlat->clear();
    surfPointsLessFlatDS->clear();

    if (skipFrameCount >= skipFrameNum)
    {
        skipFrameCount = 0;

        pcl::PointCloud<pcl::PointXYZHSV>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZHSV>(4, 1));
        setImuTrans(imuTrans);


        pcl::toROSMsg(*laserCloudExtreCur + *imuTrans, laserCloudExtreCur2);
        laserCloudExtreCur2.header.stamp = ros::Time().fromSec(timeScanCur);
        laserCloudExtreCur2.header.frame_id = "/camera";
        pubLaserCloudExtreCurPointer->publish(laserCloudExtreCur2);

        imuTrans->clear();

        pubLaserCloudLastPointer->publish(laserCloudLast2);
    }
    skipFrameCount++;
}

void scanRegistration::laserCloudHandlerVelo(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn2, pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsSharp, pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsLessSharp, pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsFlat, pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlatDS, bool pubExtre, bool pubLast)
{
    if (!systemInited) {
        initTime = laserCloudIn2->header.stamp.toSec();
        imuPointerFront = (imuPointerLast + 1) % imuQueLength;
        systemInited = true;
    }

    timeScanLast = timeScanCur;
    timeScanCur = laserCloudIn2->header.stamp.toSec();
    timeLasted = timeScanCur - initTime;

    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*laserCloudIn2, *laserCloudIn);

    std::cout << "laserCloudIn = " << laserCloudIn->size() << std::endl;

    // Take input cloud at copy it to PCL but only if inside of circle of 0.5m
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZHSV>());
    int cloudSize = createInsidePC(laserCloudIn,laserCloud);
    laserCloudIn->clear();

    std::cout << "laserCloud = " << laserCloud->size() << std::endl;


    /// Computes smoothness for each point
    computeSmoothness(laserCloud,cloudSize);

    /// Magic
    unknownFunction(laserCloud,cloudSize);

    // Compute Features
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZHSV>());
    compFeatures(cornerPointsSharp, cornerPointsLessSharp, surfPointsFlat, surfPointsLessFlat, laserCloud, cloudSize);
    std::cout << "points on sharp=" << cornerPointsSharp->size() << std::endl;
    std::cout << "points on less sharp=" << cornerPointsLessSharp->size() << std::endl;
    std::cout << "points on flat=" << surfPointsFlat->size() << std::endl;
    std::cout << "points on less flat=" << surfPointsLessFlat->size() << std::endl;

    // Filter less flat points
    pcl::VoxelGrid<pcl::PointXYZHSV> downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlat);
    downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
    downSizeFilter.filter(*surfPointsLessFlatDS);

    // copy
    *laserCloudExtreCur += *cornerPointsSharp;
    *laserCloudExtreCur += *surfPointsFlat;
    *laserCloudLessExtreCur += *cornerPointsLessSharp;
    *laserCloudLessExtreCur += *surfPointsLessFlatDS;

    // reset
    laserCloud->clear();
//    cornerPointsSharp->clear(); // Because we transport this now
//    cornerPointsLessSharp->clear();
//    surfPointsFlat->clear();
//    surfPointsLessFlat->clear();
//    surfPointsLessFlatDS->clear();


    pcl::PointCloud<pcl::PointXYZHSV>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZHSV>(4, 1));
    setImuTrans(imuTrans);

    *outLaserCloudExtreCur2 = *laserCloudExtreCur; // + *imuTrans;


    pcl::toROSMsg(*outLaserCloudExtreCur2, laserCloudExtreCur2);
    laserCloudExtreCur2.header.stamp = ros::Time().fromSec(timeScanCur);
    laserCloudExtreCur2.header.frame_id = "/camera_init_2";
    if (pubExtre)
        pubLaserCloudExtreCurPointer->publish(laserCloudExtreCur2);


    *laserCloudExtreCur += *laserCloudLessExtreCur;
    *outLaserCloudLast2 = *laserCloudExtreCur + *imuTrans;
    imuTrans->clear();
    laserCloudExtreCur->clear();
    laserCloudLessExtreCur->clear();

    pcl::toROSMsg(*outLaserCloudLast2, laserCloudLast2);
    laserCloudLast2.header.stamp = ros::Time().fromSec(timeScanLast);
    laserCloudLast2.header.frame_id = "/camera_init_2";
    if (pubLast)
        pubLaserCloudLastPointer->publish(laserCloudLast2);



}

void scanRegistration::imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
{
    double roll, pitch, yaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(imuIn->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    int imuPointerBack = imuPointerLast;
    imuPointerLast = (imuPointerLast + 1) % imuQueLength;
    imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
    double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];

    if (timeDiff < 0.1) {

        //imuAccuRoll += timeDiff * imuIn->angular_velocity.x;
        //imuAccuPitch += timeDiff * imuIn->angular_velocity.y;
        imuAccuYaw += timeDiff * imuIn->angular_velocity.z;

        imuRoll[imuPointerLast] = roll;
        imuPitch[imuPointerLast] = -pitch;
        //imuYaw[imuPointerLast] = -yaw;
        //imuRoll[imuPointerLast] = imuAccuRoll;
        //imuPitch[imuPointerLast] = -imuAccuPitch;
        imuYaw[imuPointerLast] = -imuAccuYaw;

        //imuAccX[imuPointerLast] = -imuIn->linear_acceleration.y;
        //imuAccY[imuPointerLast] = -imuIn->linear_acceleration.z - 9.81;
        //imuAccZ[imuPointerLast] = imuIn->linear_acceleration.x;

        AccumulateIMUShift();
    }
}



// Take input cloud at copy it to PCL but only if inside of circle of 0.5m
int scanRegistration::createInsidePC(const pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn, pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloud)
{
    int cloudSize = 0;
    for (int i = 0; i < laserCloudIn->points.size(); i++) {
        pcl::PointXYZHSV laserPointIn;
        laserPointIn.x = laserCloudIn->points[i].x;
        laserPointIn.y = laserCloudIn->points[i].y;
        laserPointIn.z = laserCloudIn->points[i].z;
        laserPointIn.h = timeLasted;
        laserPointIn.v = 0;

        if (!(fabs(laserPointIn.x) < 0.5 && fabs(laserPointIn.y) < 0.8 && fabs(laserPointIn.z) < 0.8) & cloudSize<CLOUD)
        {
            laserCloud->push_back(laserPointIn);
            cloudSortInd[cloudSize] = cloudSize;
            cloudNeighborPicked[cloudSize] = 0;
            cloudSize++;
        }
    }
    return cloudSize;
}

float scanRegistration::calcLaserAngle(const pcl::PointXYZ inLaserPointFirst, const pcl::PointXYZ inLaserPointLast)
{
    pcl::PointXYZ laserPointFirst = inLaserPointFirst;
    pcl::PointXYZ laserPointLast = inLaserPointLast;
    float rangeFirst = sqrt(laserPointFirst.x * laserPointFirst.x + laserPointFirst.y * laserPointFirst.y
                            + laserPointFirst.z * laserPointFirst.z);
    laserPointFirst.x /= rangeFirst;
    laserPointFirst.y /= rangeFirst;
    laserPointFirst.z /= rangeFirst;

    float rangeLast = sqrt(laserPointLast.x * laserPointLast.x + laserPointLast.y * laserPointLast.y
                           + laserPointLast.z * laserPointLast.z);
    laserPointLast.x /= rangeLast;
    laserPointLast.y /= rangeLast;
    laserPointLast.z /= rangeLast;

    float laserAngle = atan2(laserPointLast.x - laserPointFirst.x, laserPointLast.y - laserPointFirst.y);
    return laserAngle;
}

void scanRegistration::computeSmoothness(pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloud, int cloudSize)
{
    for (int i = 5; i < cloudSize - 5; i++) {
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x
                + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x
                + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x
                + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x
                + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x
                + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y
                + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y
                + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y
                + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y
                + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y
                + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z
                + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z
                + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z
                + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z
                + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z
                + laserCloud->points[i + 5].z;

        laserCloud->points[i].s = diffX * diffX + diffY * diffY + diffZ * diffZ;
    }
}

void scanRegistration::setImuTrans(pcl::PointCloud<pcl::PointXYZHSV>::Ptr imuTrans)
{
    imuTrans->points[0].x = imuPitchStart;
    imuTrans->points[0].y = imuYawStart;
    imuTrans->points[0].z = imuRollStart;
    imuTrans->points[0].v = 10;

    imuTrans->points[1].x = imuPitchCur;
    imuTrans->points[1].y = imuYawCur;
    imuTrans->points[1].z = imuRollCur;
    imuTrans->points[1].v = 11;

    imuTrans->points[2].x = imuShiftFromStartXCur;
    imuTrans->points[2].y = imuShiftFromStartYCur;
    imuTrans->points[2].z = imuShiftFromStartZCur;
    imuTrans->points[2].v = 12;

    imuTrans->points[3].x = imuVeloFromStartXCur;
    imuTrans->points[3].y = imuVeloFromStartYCur;
    imuTrans->points[3].z = imuVeloFromStartZCur;
    imuTrans->points[3].v = 13;
}

bool scanRegistration::isNewSweep(pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn)
{
    int cloudInSize = laserCloudIn->points.size();
    float laserAngle = calcLaserAngle(laserCloudIn->points[0],laserCloudIn->points[cloudInSize - 1]);
    std::cout << "laserAngle=" << laserAngle << std::endl;
    bool newSweep = false;
    if (laserAngle * laserRotDir < 0 && timeLasted - timeStart > 0.7) {
        laserRotDir *= -1;
        newSweep = true;
        std::cout << "new sweep" << std::endl;
    }
    return newSweep;
}

void scanRegistration::unknownFunction(pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloud, int cloudSize)
{
    for (int i = 5; i < cloudSize - 6; i++) {
        float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
        float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
        float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
        float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

        if (diff > 0.05) {

            float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
                                laserCloud->points[i].y * laserCloud->points[i].y +
                                laserCloud->points[i].z * laserCloud->points[i].z);

            float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x +
                    laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
                    laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);

            if (depth1 > depth2) {
                diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1;
                diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1;
                diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;

                if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {
                    cloudNeighborPicked[i - 5] = 1;
                    cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }
            } else {
                diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
                diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
                diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;

                if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    cloudNeighborPicked[i + 4] = 1;
                    cloudNeighborPicked[i + 5] = 1;
                    cloudNeighborPicked[i + 6] = 1;
                }
            }
        }

        float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
        float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
        float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
        float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

        float dis = laserCloud->points[i].x * laserCloud->points[i].x
                + laserCloud->points[i].y * laserCloud->points[i].y
                + laserCloud->points[i].z * laserCloud->points[i].z;

        if (diff > (0.25 * 0.25) / (20 * 20) * dis && diff2 > (0.25 * 0.25) / (20 * 20) * dis) {
            cloudNeighborPicked[i] = 1;
        }
    }
}

void scanRegistration::compFeatures(pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsSharp, pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsLessSharp, pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsFlat, pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlat, pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloud, int cloudSize)
{
    int startPoints[4] = {5, 6 + int((cloudSize - 10) / 4.0),
                          6 + int((cloudSize - 10) / 2.0), 6 + int(3 * (cloudSize - 10) / 4.0)};
    int endPoints[4] = {5 + int((cloudSize - 10) / 4.0), 5 + int((cloudSize - 10) / 2.0),
                        5 + int(3 * (cloudSize - 10) / 4.0), cloudSize - 6};

    for (int i = 0; i < 4; i++)
    {
        int sp = startPoints[i];
        int ep = endPoints[i];


        // sorts points based on smoothness
        for (int j = sp + 1; j <= ep; j++) {
            for (int k = j; k >= sp + 1; k--) {
                if (laserCloud->points[cloudSortInd[k]].s < laserCloud->points[cloudSortInd[k - 1]].s) {
                    int temp = cloudSortInd[k - 1];
                    cloudSortInd[k - 1] = cloudSortInd[k];
                    cloudSortInd[k] = temp;
                }
            }
        }

        int largestPickedNum = 0;
        for (int j = ep; j >= sp; j--) {
            if (cloudNeighborPicked[cloudSortInd[j]] == 0 &&
                    laserCloud->points[cloudSortInd[j]].s > 0.1 &&
                    (fabs(laserCloud->points[cloudSortInd[j]].x) > minRangeFeaturesCorners ||
                     fabs(laserCloud->points[cloudSortInd[j]].y) > minRangeFeaturesCorners ||
                     fabs(laserCloud->points[cloudSortInd[j]].z) > minRangeFeaturesCorners) &&
                    fabs(laserCloud->points[cloudSortInd[j]].x) < maxRangeFeaturesCorners &&
                    fabs(laserCloud->points[cloudSortInd[j]].y) < maxRangeFeaturesCorners &&
                    fabs(laserCloud->points[cloudSortInd[j]].z) < maxRangeFeaturesCorners) {

                largestPickedNum++;
                if (largestPickedNum <= LARGESTPICK) {
                    laserCloud->points[cloudSortInd[j]].v = 2;
                    cornerPointsSharp->push_back(laserCloud->points[cloudSortInd[j]]);
                } else if (largestPickedNum <= LARGESTPICK_SEC) {
                    laserCloud->points[cloudSortInd[j]].v = 1;
                    cornerPointsLessSharp->push_back(laserCloud->points[cloudSortInd[j]]);
                } else {
                    break;
                }

                cloudNeighborPicked[cloudSortInd[j]] = 1;
                for (int k = 1; k <= 5; k++) {
                    float diffX = laserCloud->points[cloudSortInd[j] + k].x
                            - laserCloud->points[cloudSortInd[j] + k - 1].x;
                    float diffY = laserCloud->points[cloudSortInd[j] + k].y
                            - laserCloud->points[cloudSortInd[j] + k - 1].y;
                    float diffZ = laserCloud->points[cloudSortInd[j] + k].z
                            - laserCloud->points[cloudSortInd[j] + k - 1].z;
                    if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                        break;
                    }

                    cloudNeighborPicked[cloudSortInd[j] + k] = 1;
                }
                for (int k = -1; k >= -5; k--) {
                    float diffX = laserCloud->points[cloudSortInd[j] + k].x
                            - laserCloud->points[cloudSortInd[j] + k + 1].x;
                    float diffY = laserCloud->points[cloudSortInd[j] + k].y
                            - laserCloud->points[cloudSortInd[j] + k + 1].y;
                    float diffZ = laserCloud->points[cloudSortInd[j] + k].z
                            - laserCloud->points[cloudSortInd[j] + k + 1].z;
                    if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                        break;
                    }

                    cloudNeighborPicked[cloudSortInd[j] + k] = 1;
                }
            }
        }

        int smallestPickedNum = 0;
        for (int j = sp; j <= ep; j++) {
            if (cloudNeighborPicked[cloudSortInd[j]] == 0 &&
                    laserCloud->points[cloudSortInd[j]].s < 0.1 &&
                    (fabs(laserCloud->points[cloudSortInd[j]].x) > minRangeFeaturesFlat ||
                     fabs(laserCloud->points[cloudSortInd[j]].y) > minRangeFeaturesFlat ||
                     fabs(laserCloud->points[cloudSortInd[j]].z) > minRangeFeaturesFlat) &&
                    fabs(laserCloud->points[cloudSortInd[j]].x) < maxRangeFeaturesFlat &&
                    fabs(laserCloud->points[cloudSortInd[j]].y) < maxRangeFeaturesFlat &&
                    fabs(laserCloud->points[cloudSortInd[j]].z) < maxRangeFeaturesFlat) {

                laserCloud->points[cloudSortInd[j]].v = -1;
                surfPointsFlat->push_back(laserCloud->points[cloudSortInd[j]]);

                smallestPickedNum++;
                if (smallestPickedNum >= PICKSMALL) {
                    break;
                }

                cloudNeighborPicked[cloudSortInd[j]] = 1;
                for (int k = 1; k <= 5; k++) {
                    float diffX = laserCloud->points[cloudSortInd[j] + k].x
                            - laserCloud->points[cloudSortInd[j] + k - 1].x;
                    float diffY = laserCloud->points[cloudSortInd[j] + k].y
                            - laserCloud->points[cloudSortInd[j] + k - 1].y;
                    float diffZ = laserCloud->points[cloudSortInd[j] + k].z
                            - laserCloud->points[cloudSortInd[j] + k - 1].z;
                    if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                        break;
                    }

                    cloudNeighborPicked[cloudSortInd[j] + k] = 1;
                }
                for (int k = -1; k >= -5; k--) {
                    float diffX = laserCloud->points[cloudSortInd[j] + k].x
                            - laserCloud->points[cloudSortInd[j] + k + 1].x;
                    float diffY = laserCloud->points[cloudSortInd[j] + k].y
                            - laserCloud->points[cloudSortInd[j] + k + 1].y;
                    float diffZ = laserCloud->points[cloudSortInd[j] + k].z
                            - laserCloud->points[cloudSortInd[j] + k + 1].z;
                    if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                        break;
                    }

                    cloudNeighborPicked[cloudSortInd[j] + k] = 1;
                }
            }
        }
    }

    // This are the remaing ones because v is initiallized with 0
    for (int i = 0; i < cloudSize; i++) {
        if (laserCloud->points[i].v == 0) {
            surfPointsLessFlat->push_back(laserCloud->points[i]);
        }
    }
}

void scanRegistration::haveSweep()
{
    // timeStart = timeScanLast - initTime;

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZHSV>(4, 1));
    setImuTrans(imuTrans);

    *laserCloudExtreCur += *laserCloudLessExtreCur;
    pcl::toROSMsg(*laserCloudExtreCur + *imuTrans, laserCloudLast2);
    imuTrans->clear();
    laserCloudLast2.header.stamp = ros::Time().fromSec(timeScanLast);
    laserCloudLast2.header.frame_id = "/camera";
    laserCloudExtreCur->clear();
    laserCloudLessExtreCur->clear();


    //        imuRollStart = imuRollCur;
    //        imuPitchStart = imuPitchCur;
    //        imuYawStart = imuYawCur;

    //        imuVeloXStart = imuVeloXCur;
    //        imuVeloYStart = imuVeloYCur;
    //        imuVeloZStart = imuVeloZCur;

    //        imuShiftXStart = imuShiftXCur;
    //        imuShiftYStart = imuShiftYCur;
    //        imuShiftZStart = imuShiftZCur;
}

void scanRegistration::unknownImuStuff(pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloud, int cloudSize)
{
    imuRollCur = 0; imuPitchCur = 0; imuYawCur = 0;
    imuVeloXCur = 0; imuVeloYCur = 0; imuVeloZCur = 0;
    imuShiftXCur = 0; imuShiftYCur = 0; imuShiftZCur = 0;
    if (imuPointerLast >= 0) {
        while (imuPointerFront != imuPointerLast) {
            if (timeScanCur < imuTime[imuPointerFront]) {
                break;
            }
            imuPointerFront = (imuPointerFront + 1) % imuQueLength;
        }

        if (timeScanCur > imuTime[imuPointerFront]) {
            imuRollCur = imuRoll[imuPointerFront];
            imuPitchCur = imuPitch[imuPointerFront];
            imuYawCur = imuYaw[imuPointerFront];

            imuVeloXCur = imuVeloX[imuPointerFront];
            imuVeloYCur = imuVeloY[imuPointerFront];
            imuVeloZCur = imuVeloZ[imuPointerFront];

            imuShiftXCur = imuShiftX[imuPointerFront];
            imuShiftYCur = imuShiftY[imuPointerFront];
            imuShiftZCur = imuShiftZ[imuPointerFront];
        } else {
            int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
            float ratioFront = (timeScanCur - imuTime[imuPointerBack])
                    / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            float ratioBack = (imuTime[imuPointerFront] - timeScanCur)
                    / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

            imuRollCur = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
            imuPitchCur = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
            if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > PI) {
                imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] + 2 * PI) * ratioBack;
            } else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -PI) {
                imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] - 2 * PI) * ratioBack;
            } else {
                imuYawCur = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;
            }

            imuVeloXCur = imuVeloX[imuPointerFront] * ratioFront + imuVeloX[imuPointerBack] * ratioBack;
            imuVeloYCur = imuVeloY[imuPointerFront] * ratioFront + imuVeloY[imuPointerBack] * ratioBack;
            imuVeloZCur = imuVeloZ[imuPointerFront] * ratioFront + imuVeloZ[imuPointerBack] * ratioBack;

            imuShiftXCur = imuShiftX[imuPointerFront] * ratioFront + imuShiftX[imuPointerBack] * ratioBack;
            imuShiftYCur = imuShiftY[imuPointerFront] * ratioFront + imuShiftY[imuPointerBack] * ratioBack;
            imuShiftZCur = imuShiftZ[imuPointerFront] * ratioFront + imuShiftZ[imuPointerBack] * ratioBack;
        }
    }

    if (!imuInited) {
        imuRollStart = imuRollCur;
        imuPitchStart = imuPitchCur;
        imuYawStart = imuYawCur;

        imuVeloXStart = imuVeloXCur;
        imuVeloYStart = imuVeloYCur;
        imuVeloZStart = imuVeloZCur;

        imuShiftXStart = imuShiftXCur;
        imuShiftYStart = imuShiftYCur;
        imuShiftZStart = imuShiftZCur;

        imuInited = true;
    }

    imuShiftFromStartXCur = imuShiftXCur - imuShiftXStart - imuVeloXStart * (timeLasted - timeStart);
    imuShiftFromStartYCur = imuShiftYCur - imuShiftYStart - imuVeloYStart * (timeLasted - timeStart);
    imuShiftFromStartZCur = imuShiftZCur - imuShiftZStart - imuVeloZStart * (timeLasted - timeStart);

    ShiftToStartIMU();

    imuVeloFromStartXCur = imuVeloXCur - imuVeloXStart;
    imuVeloFromStartYCur = imuVeloYCur - imuVeloYStart;
    imuVeloFromStartZCur = imuVeloZCur - imuVeloZStart;

    VeloToStartIMU();

    for (int i = 0; i < cloudSize; i++) {
        TransformToStartIMU(&laserCloud->points[i]);
    }
}

}

