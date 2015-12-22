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
#include "cv_bridge/cv_bridge.h"
#define foreach BOOST_FOREACH

#include "IOWrapper/TimestampedObject.h"
#include "DataSLAMWrapper.h"
#include "IOWrapper/InputImageStream.h"
#include "IOWrapper/ROS/ROSImageStreamThread.h"
#include "IOWrapper/ROS/ROSOutputWrapper.h"

#include "loam/loam_wrapper.h"
#include "loam/transformMaintenance.h"
#include "loam/laserMapping.h"
#include "loam/scanRegistration.h"
#include "loam/laserOdometry.h"

#include "laserscan_to_pointcloud/scanToPointCloud2.h"

//#include <X11/Xlib.h>

using namespace vl_slam;

struct data
{
    nav_msgs::OdometryPtr * ptr_od;
    sensor_msgs::PointCloud2Ptr * ptr_pc;
    sensor_msgs::ImagePtr * ptr_img;
};




int main( int argc, char** argv )
{
    //XInitThreads();
    ros::init(argc, argv, "LOAM");

    vl_slam::OutputWrapper* outputWrapper = new vl_slam::ROSOutputWrapper();
    // #############
    rosbag::Bag bag;
    bag.open("/home/sebastian/Dropbox/aria/Zhang_data_for_loam/robot_city_bridge/robot_city_bridge.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("/sync_scan_cloud_filtered"));



    rosbag::View view(bag, rosbag::TopicQuery(topics));



    // extract data from bag file

    std::vector<sensor_msgs::PointCloud2Ptr> vec_lidar;

    ros::NodeHandle nh;


    ros::Publisher pubLaserOdometry2 = nh.advertise<nav_msgs::Odometry> ("/cam_to_init_2", 5);
    tf::TransformBroadcaster tfBroadcaster2;


    ros::Publisher pubOdomBefMapped = nh.advertise<nav_msgs::Odometry> ("/bef_mapped_to_init_2", 5);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init_2", 5);
    ros::Publisher pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 1);
    transformMaintenance::transformMaintenance transformMain(&pubLaserOdometry2,&tfBroadcaster2);
    laserMapping::laserMapping laserMap(&pubOdomBefMapped,&pubOdomAftMapped,&pubLaserCloudSurround);


    ros::Publisher pubLaserCloudExtreCur = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_extre_cur", 2);

    ros::Publisher pubLaserCloudLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_last", 2);

    scanRegistration::scanRegistration scanReg(&pubLaserCloudExtreCur,&pubLaserCloudLast);

    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform laserOdometryTrans;

    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry> ("/cam_to_init", 5);
    ros::Publisher pubLaserCloudLast2 = nh.advertise<sensor_msgs::PointCloud2> ("/laser_cloud_last_2", 2);
    laserOdometry::laserOdometry laserOd(&tfBroadcaster,&laserOdometryTrans,&pubLaserOdometry,&pubLaserCloudLast2);

    int id = 1;
    foreach(rosbag::MessageInstance const m, view)
    {


        sensor_msgs::PointCloud2Ptr pc = m.instantiate<sensor_msgs::PointCloud2>();
        if (pc != NULL)
        {
            // publish raw scan
//            nav_msgs::Odometry inputOdometry;
//            SE3 imagePose;
//            imagePose.translation()[0] = 0;
//            imagePose.translation()[1] = 0;
//            imagePose.translation()[2] = 0;
//            inputOdometry.pose.pose = imagePose.getAsGeoPose();
            //            outputWrapper->publishKeyframe(*pc,inputOdometry,id);
            //            id++;

            // Registration
            sensor_msgs::PointCloud2 outExtreCur2;
            sensor_msgs::PointCloud2 outCloudLast2;
            scanReg.laserCloudHandler(pc,outExtreCur2,outCloudLast2);
            if(outCloudLast2.width>0)
            {
                laserOd.laserCloudLastHandler(outCloudLast2);
                outCloudLast2.fields[3].name = "intensity";
            }
            if(outExtreCur2.width>0)
            {
                 laserOd.laserCloudExtreCurHandler(outExtreCur2);
            }



            sensor_msgs::PointCloud2 pub;
            nav_msgs::Odometry pubOdo;
            laserOd.main_laserOdometry(pub, pubOdo);


 std::cout << "pubOdo.pose.pose.position=" << pubOdo.pose.pose.position << std::endl;

            //if (!std::isnan(pubOdo.pose.pose.position.x))
            //{
                laserMap.laserOdometryHandler(pubOdo);
            //}
            //else
            //{

            //}


            if (pub.width>0)
            {
                //                    pub.fields[3].name = "intensity";
                laserMap.laserCloudLastHandler(pub);

            }


            sensor_msgs::PointCloud2 laser_cloud_surround;
            nav_msgs::Odometry odomBefMapped;
            nav_msgs::Odometry odomAftMapped;
            laserMap.loop(laser_cloud_surround, odomBefMapped, odomAftMapped);

            // maintanance
            nav_msgs::Odometry outlaserOdometry2;

            transformMain.odomAftMappedHandler(odomAftMapped);
            transformMain.odomBefMappedHandler(odomBefMapped);
            transformMain.laserOdometryHandler(pubOdo,outlaserOdometry2);



            std::cout << "pubOdo.pose.pose.position=" << pubOdo.pose.pose.position << std::endl;
            std::cout << "odomAftMapped.pose.pose.position=" << odomAftMapped.pose.pose.position << std::endl;
            std::cout << "outlaserOdometry2.pose.pose.position=" << outlaserOdometry2.pose.pose.position << std::endl;
            //cv::waitKey(100);
            outputWrapper->publishKeyframe(laser_cloud_surround,outlaserOdometry2,id);
            id++;













        }
    }


    std::cout << "vec_lidar.size()=" << vec_lidar.size() << std::endl;



    //    //sort data by timestamp
    //    std::vector<data> data_stream;
    //    for (int i=0,j=0,k=0,l=0;i<vec_odometry.size() & j<vec_lidar.size() & k<vec_images.size() & l<(vec_odometry.size()+vec_lidar.size()+vec_images.size());l++)
    //    {
    //        data temp;
    //        if(vec_odometry[i].get()->header.stamp.toSec() < vec_lidar[j]->header.stamp.toSec())
    //        {
    //            if(vec_odometry[i].get()->header.stamp.toSec() <= vec_images[k].get()->header.stamp.toSec())
    //            {
    //                temp.ptr_img = 0;
    //                temp.ptr_od = &vec_odometry[i];
    //                temp.ptr_pc = 0;
    //                i++;
    //                //std::cout << "i=" << i << std::endl;
    //                data_stream.push_back(temp);
    //                continue;
    //            }
    //        }
    //        if(vec_lidar[j].get()->header.stamp.toSec() < vec_odometry[i]->header.stamp.toSec())
    //        {
    //            if(vec_lidar[j].get()->header.stamp.toSec() < vec_images[k].get()->header.stamp.toSec())
    //            {
    //                temp.ptr_img = 0;
    //                temp.ptr_od = &vec_odometry[i];
    //                temp.ptr_pc = &vec_lidar[j];
    //                j++;
    //                //std::cout << "j=" << j << std::endl;
    //                data_stream.push_back(temp);
    //                continue;
    //            }
    //        }
    //        if(vec_images[k].get()->header.stamp.toSec() < vec_lidar[j]->header.stamp.toSec())
    //        {
    //            if(vec_images[k].get()->header.stamp.toSec() < vec_odometry[i].get()->header.stamp.toSec())
    //            {
    //                temp.ptr_img = &vec_images[k];
    //                temp.ptr_od = 0;
    //                temp.ptr_pc = 0;
    //                k++;
    //                //std::cout << "k=" << k << std::endl;
    //                data_stream.push_back(temp);
    //                continue;
    //            }
    //        }

    //    }
    //    std::cout << "data_stream.size()=" << data_stream.size() << std::endl;


    //    cv::Mat image = cv::Mat(h,w,CV_8U);
    //    int lastID=0;
    //    for (int i=0;i<data_stream.size();i++)
    //    {
    //        if(data_stream[i].ptr_pc != 0 && data_stream[i].ptr_od != 0)
    //        {
    //            TimestampedPointCloud2 lidarData = lidarCb(*data_stream[i].ptr_pc);
    //            lastID=slamNode.newLidarCallback(lidarData,*data_stream[i].ptr_od->get());
    //        }
    //        if(data_stream[i].ptr_img != 0)
    //        {
    //            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*data_stream[i].ptr_img, sensor_msgs::image_encodings::MONO8);
    //            cv::Mat cvimg  = cv_ptr->image;
    //            //            cv::imshow("fff",cvimg);
    //            undistorter->undistort(cvimg, image);
    //            slamNode.newImageCallback(lastID+1,image, Timestamp((*data_stream[i].ptr_img)->header.stamp.toSec()));
    //        }
    //        if(data_stream[i].ptr_od == 0)
    //        {
    //            //            TimestampedPointCloud2 lidarData = lidarCb(*data_stream[i].ptr_pc);
    //            //            slamNode.newLidarCallback(lidarData,lastOd);
    //        }
    //    }




    //    // Do local BA
    //    slamNode.localBAThreadLoop();

    // Loop closure
    //slamNode.loopClosureSearchLoop();

    // upsampling
    //    double * depth = new double[width*height];
    //    for (size_t u=0;u<width;u++)
    //        for (size_t v=0;v<height;v++)
    //            depth[u+v*w] = -1;

    //    cv::Mat img;
    //    SE3 imagePose;
    //    slamNode.getDepthImage(depth,2,2,img,imagePose);


    //    std::pair<cv::Mat,double*> data;
    //    data.first = img;
    //    data.second = depth;



    //    double *depth_result = new double[width*height];
    //    doSMO_single_gray(data,depth_result);
    //    pcl::PointCloud<pcl::PointXYZI>::Ptr msg (new pcl::PointCloud<pcl::PointXYZI>);
    //    msg->header.frame_id = "some_tf_frame";
    //    msg->height = 1;
    //    cv::Mat displayLidarImage = cv::Mat(height,width, CV_8UC3);
    //    cv::cvtColor(img, displayLidarImage, CV_GRAY2RGB);
    //    for (int y=0;y<height;y++)
    //        for (int x=0;x<width;x++)
    //        {
    //            //std::cout << "x+y*width_s = " <<  x+y*width_s << std::endl;


    //            float id = depth_result[x+y*width];
    //            if (id>0)
    //            {

    //                // rainbow between 0 and 4
    //                float r = (1) * 255 / 1; if(r < 0) r = -r;
    //                float g = (2-id) * 255 / 1; if(g < 0) g = -g;
    //                float b = (3-id) * 255 / 1; if(b < 0) b = -b;

    //                uchar rc = r < 0 ? 0 : (r > 255 ? 255 : r);
    //                uchar gc = g < 0 ? 0 : (g > 255 ? 255 : g);
    //                uchar bc = b < 0 ? 0 : (b > 255 ? 255 : b);

    //                cv::Scalar color = cv::Scalar(255-rc,255-gc,255-bc);
    //                cv::circle(displayLidarImage, cv::Point2f(x, y), 1, color, 1, 8,0);

    //                pcl::PointXYZI temp(1.0);
    //                Eigen::Vector3d uv;
    //                uv << x , y , id;
    //                Eigen::Vector3d xyz;
    //                xyz = K.inverse()*uv;
    //                temp.x = xyz(0);
    //                temp.y = xyz(1);
    //                temp.z = id;
    //                msg->points.push_back (temp);
    //            }

    //        }
    //    msg->width = msg->points.size();




    //    sensor_msgs::PointCloud2 inPC;
    //    pcl::toROSMsg(*msg,inPC);

    //    cv::imshow("depth map",displayLidarImage);
    //    cv::waitKey(100000);

    //    nav_msgs::Odometry inputOdometry;
    //    inputOdometry.pose.pose = imagePose.getAsGeoPose();
    //    int id = 1;
    //    outputWrapper->publishUpsampling(inPC,inputOdometry,id);
    //    id=0;
    //    outputWrapper->publishKeyframeImage(inPC,inputOdometry,id);
    //    id=2;
    //    ImageFrame * out_img = new ImageFrame(1, width, height, K.cast<float>(), 0, img.data);
    //    float * out_depth = new float[width*height];
    //    for (int i=0;i<(width*height);i++)
    //        out_depth[i] = depth_result[i];
    //    out_img->setDepthFromGroundTruth(out_depth);
    //    ImageFramePoseStruct out_pose(out_img);
    //    out_pose.isInGraph = true;
    //    out_pose.setPoseGraphOptResult(imagePose);
    //    out_pose.applyPoseGraphOptResult();
    //    out_img->pose = &out_pose;



    //    out_img->status = 1;
    //    outputWrapper->publishKeyframeDepth(out_img);



    //    for (int i=0;i<(width*height);i++)
    //        out_depth[i] = depth[i];
    //    out_img = new ImageFrame(2, width, height, K.cast<float>(), 0, img.data);
    //    out_img->setDepthFromGroundTruth(out_depth);
    //    out_img->status = 2;
    //    out_pose.isInGraph = true;
    //    out_pose.setPoseGraphOptResult(imagePose);
    //    out_pose.applyPoseGraphOptResult();
    //    out_img->pose = &out_pose;
    //    outputWrapper->publishKeyframeDepth(out_img);

    //  slamNode.depthEstimationLoop();




    //   bag.close();
    return 0;
}
