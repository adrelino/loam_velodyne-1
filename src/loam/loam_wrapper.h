//namespace scanRegistration {
//void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn2,  sensor_msgs::PointCloud2 &outExtreCur2, sensor_msgs::PointCloud2 &outCloudLast2);
//}



namespace laserOdometry {
void laserCloudExtreCurHandler(const sensor_msgs::PointCloud2& laserCloudExtreCur2);
void laserCloudLastHandler(const sensor_msgs::PointCloud2& laserCloudLast2);
void main_laserOdometry(sensor_msgs::PointCloud2 &pub, nav_msgs::Odometry &pubOdo);
}

namespace laserMapping {
//void laserCloudLastHandler(const sensor_msgs::PointCloud2& laserCloudLast2);
//void laserOdometryHandler(const nav_msgs::Odometry& laserOdometry);
//void main(sensor_msgs::PointCloud2 &laser_cloud_surround, nav_msgs::Odometry & odomBefMapped,nav_msgs::Odometry & odomAftMapped);
}

namespace transformMaintenance {
//void laserOdometryHandler(const nav_msgs::Odometry& laserOdometry, nav_msgs::Odometry &outlaserOdometry2);
//void odomBefMappedHandler(const nav_msgs::Odometry& odomBefMapped);
//void odomAftMappedHandler(const nav_msgs::Odometry& odomAftMapped);
//int main();




}
