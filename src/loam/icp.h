#pragma once
#include <Eigen/Core>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/transforms.h>

#include "ceres/ceres.h"
#include "glog/logging.h"


#include<Eigen/SVD>




using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
// A templated cost functor that implements the residual r = 10 -
// x. The method operator() is templated so that we can then use an
// automatic differentiation wrapper around it to generate its
// derivatives.
struct CostFunctor {
    template <typename T> bool operator()(const T* const x, T* residual) const {
        residual[0] = T(10.0) - x[0];
        return true;
    }
};

class ICP
{
public:
    ICP();
    void associate();
    void nonlinearLVM();
    void testCovariance();
    void testLVM();
    bool associatePointToPlane(pcl::PointXYZ &searchPoint, pcl::PointXYZ &normal, pcl::PointXYZ &point);
    Eigen::MatrixXf getPlaneJacobi(float * theta, pcl::PointXYZ normal, pcl::PointXYZ normalPosition, pcl::PointXYZ point_k1);
    float getPoint2PlaneDistance(pcl::PointXYZ normal, pcl::PointXYZ normalPosition, pcl::PointXYZ point);
    void computeMean(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, std::vector<int> & pointIdxNKNSearch, pcl::PointXYZ & mean);
    void computeCovariance(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, std::vector<int> & pointIdxNKNSearch, pcl::PointXYZ mean, Eigen::Matrix3f & covariance);
    void doICP();
    void setKdTree(pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudSurfFromMap)
    {
        kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);
    }

    void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr in)
    {
        *inputCloud = *in;
    }

    void setSurfaceMap(pcl::PointCloud<pcl::PointXYZ>::Ptr in)
    {
        *surfaceMap = *in;
        setKdTree(surfaceMap);
    }

    Eigen::Matrix3f getRotationMatrix(float phi, float theta, float psi)
    {
        Eigen::Matrix3f R;
        R << cos(phi)*cos(theta), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi),
                sin(phi)*cos(theta), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),
                -sin(theta), cos(theta)*sin(psi), cos(theta)*cos(psi);
        return R;
    }

    Eigen::Matrix4f getTransformationMatrix()
    {
        Eigen::Matrix4f T;
        Eigen::Matrix3f R;
        R = getRotationMatrix(transformation[3],transformation[4],transformation[5]);
        T << R(0,0), R(0,1), R(0,2), transformation[0],
             R(1,0), R(1,1), R(1,2), transformation[1],
             R(2,0), R(2,1), R(2,2), transformation[2],
             0, 0, 0, 1;
        return T;
    }

private:
    int maxIteration = 20;
    int K = 15;
    float transformation[6];

    // HSV
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud;

    pcl::PointCloud<pcl::PointXYZ>::Ptr surfaceMap;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeSurfFromMap;

    std::vector<int> pointToPlaneIdx;
    pcl::PointCloud<pcl::PointXYZ>::Ptr surfaceNormals;
    pcl::PointCloud<pcl::PointXYZ>::Ptr surfaceNormalsPosition;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedInputCloud;

    float w_11(float phi, float theta, float psi)
    {
        return cos(phi) * cos(theta);
    }

    float w_12(float phi, float theta, float psi)
    {
        return cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi);
    }

    float w_13(float phi, float theta, float psi)
    {
        return cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi);
    }

    float w_21(float phi, float theta, float psi)
    {
        return sin(phi) * cos(theta);
    }

    float w_22(float phi, float theta, float psi)
    {
        return sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi);
    }

    float w_23(float phi, float theta, float psi)
    {
        return sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi);
    }

    float w_31(float phi,float theta,float psi)
    {
        return -sin(theta);
    }

    float w_32(float phi,float theta,float psi)
    {
        return cos(theta) * sin(psi);
    }

    float w_33(float phi,float theta,float psi)
    {
        return cos(theta) * cos(psi);
    }

    float w_11_phi(float phi,float theta,float psi)
    {
        return -w_21(phi,theta,psi);
    }

    float w_11_theta(float phi,float theta,float psi)
    {
        return w_31(phi,theta,psi) * cos(phi);
    }

    float w_11_psi(float phi,float theta,float psi)
    {
        return 0;
    }

    float w_12_phi(float phi,float theta,float psi)
    {
        return -w_22(phi,theta,psi);
    }

    float w_12_theta(float phi,float theta,float psi)
    {
        return w_33(phi,theta,psi) * cos(phi);
    }

    float w_12_psi(float phi,float theta,float psi)
    {
        return w_13(phi,theta,psi);
    }

    float w_13_phi(float phi,float theta,float psi)
    {
        return -w_23(phi,theta,psi);
    }

    float w_13_theta(float phi,float theta,float psi)
    {
        return w_33(phi,theta,psi) * cos(phi);
    }

    float w_13_psi(float phi,float theta,float psi)
    {
        return -w_12(phi,theta,psi);
    }

    float w_21_phi(float phi,float theta,float psi)
    {
        return w_11(phi,theta,psi);
    }

    float w_21_theta(float phi,float theta,float psi)
    {
        return w_31(phi,theta,psi) * sin(phi);
    }

    float w_21_psi(float phi,float theta,float psi)
    {
        return 0.0f;
    }

    float w_22_phi(float phi,float theta,float psi)
    {
        return w_12(phi,theta,psi);
    }

    float w_22_theta(float phi,float theta,float psi)
    {
        return w_31(phi,theta,psi) * sin(phi);
    }

    float w_22_psi(float phi,float theta,float psi)
    {
        return w_23(phi,theta,psi);
    }

    float w_23_phi(float phi,float theta,float psi)
    {
        return w_13(phi,theta,psi);
    }

    float w_23_theta(float phi,float theta,float psi)
    {
        return w_33(phi,theta,psi) * sin(phi);
    }

    float w_23_psi(float phi,float theta,float psi)
    {
        return -w_22(phi,theta,psi);
    }

    float w_31_phi(float phi,float theta,float psi)
    {
        return 0.0f;
    }

    float w_31_theta(float phi,float theta,float psi)
    {
        return -w_33(phi,theta,psi) * cos(psi);
    }

    float w_31_psi(float phi,float theta,float psi)
    {
        return 0.0f;
    }

    float w_32_phi(float phi,float theta,float psi)
    {
        return 0.0f;
    }

    float w_32_theta(float phi,float theta,float psi)
    {
        return -cos(theta);
    }

    float w_32_psi(float phi,float theta,float psi)
    {
        return w_33(phi,theta,psi);
    }

    float w_33_phi(float phi,float theta,float psi)
    {
        return 0.0f;
    }

    float w_33_theta(float phi,float theta,float psi)
    {
        return w_31(phi,theta,psi) * cos(psi);
    }

    float w_33_psi(float phi,float theta,float psi)
    {
        return -w_32(phi,theta,psi);
    }

    Eigen::Vector3f a_x_theta(float phi,float theta,float psi, Eigen::Vector3f X)
    {
        Eigen::Matrix3f temp;
        temp << w_11_phi(phi,theta,psi), w_12_phi(phi,theta,psi), w_13_phi(phi,theta,psi),
                w_11_theta(phi,theta,psi), w_12_theta(phi,theta,psi), w_13_theta(phi,theta,psi),
                w_11_psi(phi,theta,psi), w_12_psi(phi,theta,psi), w_13_psi(phi,theta,psi);
        return temp * X;
    }

    Eigen::Vector3f a_y_theta(float phi,float theta,float psi, Eigen::Vector3f X)
    {
        Eigen::Matrix3f temp;
        temp << w_21_phi(phi,theta,psi), w_22_phi(phi,theta,psi), w_23_phi(phi,theta,psi),
                w_21_theta(phi,theta,psi), w_22_theta(phi,theta,psi), w_23_theta(phi,theta,psi),
                w_21_psi(phi,theta,psi), w_22_psi(phi,theta,psi), w_23_psi(phi,theta,psi);
        return temp * X;
    }

    Eigen::Vector3f a_z_theta(float phi,float theta,float psi, Eigen::Vector3f X)
    {
        Eigen::Matrix3f temp;
        temp << w_31_phi(phi,theta,psi), w_32_phi(phi,theta,psi), w_33_phi(phi,theta,psi),
                w_31_theta(phi,theta,psi), w_32_theta(phi,theta,psi), w_33_theta(phi,theta,psi),
                w_31_psi(phi,theta,psi), w_32_psi(phi,theta,psi), w_33_psi(phi,theta,psi);
        return temp * X;
    }

    void testGetPoint2PlaneDistance()
    {
        pcl::PointXYZ normal;
        normal.x = 0;
        normal.y = 0;
        normal.z = 1;
        pcl::PointXYZ normalPosition;
        normalPosition.x = 1;
        normalPosition.y = 1;
        normalPosition.z = 1;
        pcl::PointXYZ point;
        point.x = 1;
        point.y = 1;
        point.z = 0;
        std::cout << "Should be one : " << getPoint2PlaneDistance(normal, normalPosition, point) << std::endl;

        point.x = 0;
        point.y = 1;
        point.z = 1;

        std::cout << "Should be zero : " << getPoint2PlaneDistance(normal, normalPosition, point) << std::endl;

    }

    void testGetPlaneJacobi()
    {
        float theta[6];
        for (int i=0;i<6;i++)
            theta[i] = 0.0f;

        pcl::PointXYZ normal;
        normal.x = 0;
        normal.y = 0;
        normal.z = 1;
        pcl::PointXYZ normalPosition;
        normalPosition.x = 1;
        normalPosition.y = 1;
        normalPosition.z = 1;
        pcl::PointXYZ point;
        point.x = 1;
        point.y = 1;
        point.z = 0;

        std::cout << "Should be something: " << getPlaneJacobi(theta, normal, normalPosition, point) << std::endl;

        theta[2] = 1.0f;

        std::cout << "Should be zero: " << getPlaneJacobi(theta, normal, normalPosition, point) << std::endl;

    }
};
