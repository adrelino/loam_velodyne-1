#include "icp.h"
ICP::ICP()
{
    for(int i=0;i<6;i++)
        transformation[i] = 0.0f;

    inputCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());

    surfaceMap.reset(new pcl::PointCloud<pcl::PointXYZ>());
    kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());

    surfaceNormals.reset(new pcl::PointCloud<pcl::PointXYZ>());
    surfaceNormalsPosition.reset(new pcl::PointCloud<pcl::PointXYZ>());
    transformedInputCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
}


// associates plane and computes normal and corresponding point
// can use pcl::computeMeanAndCovarianceMatrix ?
bool ICP::associatePointToPlane(pcl::PointXYZ &searchPoint, pcl::PointXYZ &normal, pcl::PointXYZ &normalPosition)
{
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    if (!(kdtreeSurfFromMap->nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0))
        return false;

    if (pointNKNSquaredDistance[K-1] > 1.0)
        return false;

    computeMean(surfaceMap, pointIdxNKNSearch, normalPosition);

    Eigen::Matrix3f covariance;
    computeCovariance(surfaceMap, pointIdxNKNSearch, normalPosition, covariance);

    Eigen::EigenSolver<Eigen::Matrix3f> es;
    es.compute(covariance,true);
    Eigen::Matrix<std::complex<float>, 3, 1>  eig = es.eigenvalues();
    Eigen::Matrix<std::complex<float>, 3, 3> eig_vec = es.eigenvectors();


    // Two eigenvalue should be significantly larger
    //std::cout << "eig(0, 0).real()"  > 3 * eig(2, 0).real() && eig(1, 0).real() > 3 * eig(2, 0).real()"
    if (eig(0, 0).real() > 3 * eig(2, 0).real() && eig(1, 0).real() > 3 * eig(2, 0).real() && fabs(eig(0, 0).real()-eig(1, 0).real() ) < 0.3)
    {
        normal.x = eig_vec(0, 2).real();
        normal.y = eig_vec(1, 2).real();
        normal.z = eig_vec(2, 2).real();
        //std::cout << "searchPoint:" << searchPoint << std::endl;
        //std::cout << "cov:" << covariance << std::endl;
        //std::cout << "lambda1=" << eig(0, 0).real() << ", lambda2=" << eig(1, 0).real() << ", lambda3=" << eig(2, 0).real() << std::endl;
        //std::cout << "eig" << eig << std::endl;
        //std::cout << "covariance=" << covariance << std::endl;
        //std::cout << "eig_vec=" << eig_vec << std::endl;
        //std::cout << "normal.x=" << normal.x << "normal.y=" << normal.y << "normal.z=" << normal.z << std::endl;
        return true;
    }
    else if (eig(2, 0).real() > 3 * eig(0, 0).real() && eig(1, 0).real() > 3 * eig(0, 0).real() && fabs(eig(2, 0).real()-eig(1, 0).real() ) < 0.3)
    {
        normal.x = eig_vec(0, 0).real();
        normal.y = eig_vec(1, 0).real();
        normal.z = eig_vec(2, 0).real();
        return true;
    }
    else if (eig(2, 0).real() > 3 * eig(1, 0).real() && eig(0, 0).real() > 3 * eig(1, 0).real() && fabs(eig(2, 0).real()-eig(0, 0).real() ) < 0.3)
    {
        normal.x = eig_vec(0, 1).real();
        normal.y = eig_vec(1, 1).real();
        normal.z = eig_vec(2, 1).real();
        return true;
    }
    else
    {
        //std::cout << "eig:" << eig << std::endl;
        return false;
    }


}

void ICP::computeMean(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, std::vector<int> & pointIdxNKNSearch, pcl::PointXYZ & mean)
{
    mean.x=0;
    mean.y=0;
    mean.z=0;
    int K = pointIdxNKNSearch.size();
    for (int i = 0; i < K; i++)
    {
        mean.x += pc->points[pointIdxNKNSearch[i]].x;
        mean.y += pc->points[pointIdxNKNSearch[i]].y;
        mean.z += pc->points[pointIdxNKNSearch[i]].z;
    }
    mean.x/=K;
    mean.y/=K;
    mean.z/=K;
}

void ICP::testCovariance()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointXYZ p1(0.4391,-1.8673, 52.7055);
    pcl::PointXYZ p2(0.4923,-1.6150, 52.5819);
    pcl::PointXYZ p3(0.5987,-1.8510, 52.1647);
    pcl::PointXYZ p4(0.3019,-2.2129, 52.9612);
    pcl::PointXYZ p5(0.9917,-1.6178, 52.6602);
    pc->points.push_back(p1);
    pc->points.push_back(p2);
    pc->points.push_back(p3);
    pc->points.push_back(p4);
    pc->points.push_back(p5);
    std::vector<int> pointIdxNKNSearch;
    pointIdxNKNSearch.push_back(0);
    pointIdxNKNSearch.push_back(1);
    pointIdxNKNSearch.push_back(2);
    pointIdxNKNSearch.push_back(3);
    pointIdxNKNSearch.push_back(4);

    pcl::PointXYZ normalPosition;
    computeMean(pc, pointIdxNKNSearch, normalPosition);
    std::cout << "Mean should be 0.5648 -1.8328 52.6147 and it is:" << normalPosition << std::endl;

    Eigen::Matrix3f cov;
    cov << 0.0547, 0.0359, -0.0192,
            0.0359, 0.0479, -0.0248,
            -0.0192, -0.0248, 0.0668;
    Eigen::Matrix3f covariance;
    computeCovariance(pc, pointIdxNKNSearch, normalPosition, covariance);
    std::cout << "Covariance should be " << cov << " and it is:" << covariance << std::endl;


    Eigen::EigenSolver<Eigen::Matrix3f> es;
    es.compute(covariance,true);
    Eigen::Matrix<std::complex<float>, 3, 1>  eig = es.eigenvalues();
    Eigen::Matrix<std::complex<float>, 3, 3> eig_vec = es.eigenvectors();


    Eigen::Matrix3f eigen_vectors;
    eigen_vectors << 0.6227, 0.5295, -0.5761,
            -0.7704, 0.2862, -0.5697,
            -0.1367, 0.7986, 0.5862;

    Eigen::Vector3f eigen_values;
    eigen_values(0) = 0.0145;
    eigen_values(1) = 0.0452;
    eigen_values(2) = 0.1098;

    std::cout << "eigen values should be " << eigen_values << " and they are: " << eig << std::endl;
    std::cout << "eigen vectors should be " << eigen_vectors << " and they are: " << eig_vec << std::endl;

}

void ICP::testLVM()
{
    surfaceNormals->points.clear();
    surfaceNormals->points.push_back(pcl::PointXYZ(-0.8574, 0.4340, -0.2768));
    surfaceNormals->points.push_back(pcl::PointXYZ(-0.5024, -0.6924, 0.5178));
    surfaceNormals->points.push_back(pcl::PointXYZ(-0.5581, 0.3522, 0.7513));
    surfaceNormals->points.push_back(pcl::PointXYZ(-0.0095, 0.9994, 0.0347));
    surfaceNormals->points.push_back(pcl::PointXYZ(0.9220, -0.1186, -0.3687));
    surfaceNormals->points.push_back(pcl::PointXYZ(0.9763, -0.2151, 0.0232));
    surfaceNormals->points.push_back(pcl::PointXYZ(0.2143, 0.1000, 0.9716));
    surfaceNormals->points.push_back(pcl::PointXYZ(-0.0008, -1.0000, -0.0028));
    surfaceNormals->points.push_back(pcl::PointXYZ(-0.7312, 0.4384, -0.5227));
    surfaceNormals->points.push_back(pcl::PointXYZ(-0.0008, -1.0000, -0.0028));

    surfaceNormalsPosition->points.clear();
    surfaceNormalsPosition->points.push_back(pcl::PointXYZ(0.4899,-1.9372, 52.2678));
    surfaceNormalsPosition->points.push_back(pcl::PointXYZ(-10.0554,-1.7153,41.5334));
    surfaceNormalsPosition->points.push_back(pcl::PointXYZ(-14.7504,-2.1679,54.2432));
    surfaceNormalsPosition->points.push_back(pcl::PointXYZ(-13.3243,-1.8150,40.4852));
    surfaceNormalsPosition->points.push_back(pcl::PointXYZ(-11.2095,-1.0897,32.0119));
    surfaceNormalsPosition->points.push_back(pcl::PointXYZ(-13.3994,-1.5460,35.5189));
    surfaceNormalsPosition->points.push_back(pcl::PointXYZ(-13.6893,-0.2492,4.5698));
    surfaceNormalsPosition->points.push_back(pcl::PointXYZ(-11.6284,-0.2389,2.9539));
    surfaceNormalsPosition->points.push_back(pcl::PointXYZ(-11.5892,-0.2231,2.8199));
    surfaceNormalsPosition->points.push_back(pcl::PointXYZ(-11.4813,-0.2375,2.4026));

    pointToPlaneIdx.clear();
    pointToPlaneIdx.push_back(0);
    pointToPlaneIdx.push_back(1);
    pointToPlaneIdx.push_back(2);
    pointToPlaneIdx.push_back(3);
    pointToPlaneIdx.push_back(4);
    pointToPlaneIdx.push_back(5);
    pointToPlaneIdx.push_back(6);
    pointToPlaneIdx.push_back(7);
    pointToPlaneIdx.push_back(8);
    pointToPlaneIdx.push_back(9);

    inputCloud->points.clear();
    inputCloud->points.push_back(pcl::PointXYZ(0.3805,-2.1881,52.2769));
    inputCloud->points.push_back(pcl::PointXYZ(-9.9498,-1.8275,41.6130));
    inputCloud->points.push_back(pcl::PointXYZ(-14.3400,-2.2863,53.6381));
    inputCloud->points.push_back(pcl::PointXYZ(-13.0278,-1.8154,40.5712));
    inputCloud->points.push_back(pcl::PointXYZ(-11.1502,-1.4768,31.3779));
    inputCloud->points.push_back(pcl::PointXYZ(-13.2939,-1.6410,35.4776));
    inputCloud->points.push_back(pcl::PointXYZ(-13.7335,-0.2361,3.9676));
    inputCloud->points.push_back(pcl::PointXYZ(-11.6497,-0.2390,2.9558));
    inputCloud->points.push_back(pcl::PointXYZ(-11.6491,-0.2385,2.7760));
    inputCloud->points.push_back(pcl::PointXYZ(-11.6150,-0.2376,2.4120));


    nonlinearLVM();

    Eigen::MatrixXf J(10,6);
    J << -0.1332, 0.1759, 0.0522, 0.5482, -7.1259, -8.4516,
            0.1022,    0.0660,    0.1780,   -1.0595,    5.9748,   -2.4864,
            0.2844,    0.4461,   -0.4863,   -7.9591,    4.4829,  -26.8760,
            0.0001,    0.5474,    0.0007,   -7.2387,   -1.5335,  -22.1145,
            0.3551,   -0.0376,   -0.0990,   -0.6528,    9.2868,    0.1605,
            0.4553,   -0.0376,    0.0002,   -1.0910,   15.2900,    0.1792,
            0.0245,   -0.0802,   -0.5927,    1.0712,   -8.0832,    0.1724,
            0.0000,   -8.5735,    0.0000,   99.8318,    1.6951,   25.3424,
            0.2656,   -1.6617,    0.1505,   19.1053,    2.7111,    4.5771,
            0.0000,   -8.7069,    0.0000,  101.0710,    1.3905,   21.0461;

    //std::cout << "J should be: " << J << std::endl;

    Eigen::MatrixXf JtJ(6,6);
    JtJ << 0.1488, -0.0005, -0.1838, 0.4956, 3.4359, 0.3623,
            -0.0005,    0.0216,    0.0171,   -0.1256,    0.3569,   -1.0411,
            -0.1838,   0.0171,   1.7914,   -0.6804,   16.8306,   -2.3559,
            0.4956,   -0.1256,   -0.6804,    2.8013,   11.2897,    6.6743,
            3.4359,    0.3569,   16.8306,   11.2897,  374.8922,  -24.3352,
            0.3623,   -1.0411,   -2.3559,    6.6743,  -24.3352,   54.0882;
    //std::cout << "JtJ should be: " << JtJ << std::endl;

    // sum d 1.0131


    Eigen::MatrixXf inv(6,6);
    inv << 178.888529759719,	274.573258377150,	81.0448315632644,	17.1659788916215,	-5.86498324972197,	2.85720659342748,
            274.573258377775,	3164.46438905935,	297.541523820153,	135.639281121808,	-19.9480017716936,	46.2738647006479,
            81.0448315633125,	297.541523820022,	51.0723570808690,	17.1076257153673,	-3.59172979699504,	3.67830418525987,
            17.1659788916509,	135.639281121833,	17.1076257153754,	8.43191479038909,	-1.19951092299947,	1.65923712718852,
            -5.86498324972510,	-19.9480017716838,	-3.59172979699493,	-1.19951092299888,	0.257128013334168,	-0.237193909448802,
            2.85720659343653,	46.2738647006473,	3.67830418526172,	1.65923712718812,	-0.237193909448942,	0.738075209372451;
    //std::cout << "inv should be:" << inv << std::endl;


    Eigen::MatrixXf up(6,1);
    up << -0.3024,
     5.4591,
    -0.4369,
     0.0705,
     0.0108,
     0.1181;

    std::cout << "inc should be:" << up << std::endl;

}

void ICP::computeCovariance(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, std::vector<int> & pointIdxNKNSearch, pcl::PointXYZ mean, Eigen::Matrix3f & covariance)
{
    float a11 = 0;
    float a12 = 0;
    float a13 = 0;
    float a22 = 0;
    float a23 = 0;
    float a33 = 0;
    int K = pointIdxNKNSearch.size();
    for (int i = 0; i < K; i++)
    {
        float ax = pc->points[pointIdxNKNSearch[i]].x - mean.x;
        float ay = pc->points[pointIdxNKNSearch[i]].y - mean.y;
        float az = pc->points[pointIdxNKNSearch[i]].z - mean.z;

        a11 += ax * ax;
        a12 += ax * ay;
        a13 += ax * az;
        a22 += ay * ay;
        a23 += ay * az;
        a33 += az * az;
    }

    covariance << a11, a12, a13, a12, a22, a23, a13, a23, a33;
    covariance = covariance / K;

}

void ICP::doICP()
{
    //    double x = 0.5;
    //    const double initial_x = x;
    //    // Build the problem.
    //    Problem problem;
    //    // Set up the only cost function (also known as residual). This uses
    //    // auto-differentiation to obtain the derivative (jacobian).
    //    CostFunction* cost_function = new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
    //    problem.AddResidualBlock(cost_function, NULL, &x);
    //    // Run the solver!
    //    Solver::Options options;

    //    options.minimizer_progress_to_stdout = true;
    //    Solver::Summary summary;
    //    Solve(options, &problem, &summary);
    //    std::cout << summary.BriefReport() << "\n";
    //    std::cout << "x : " << initial_x
    //              << " -> " << x << "\n";


    //float deltaR, deltaT;
    for (int iter= 0; iter < maxIteration; iter++)
    {
        std::cout << "[Mapping] iter: " << iter << std::endl;

        associate();

        nonlinearLVM();

        std::cout << "transformation: " << transformation[0] << " " << transformation[1] << " " << transformation[2]
                  << " " << transformation[3] << " " << transformation[4] << " " << transformation[5] << std::endl;


        //        if (deltaR < icp_R_break && deltaT < icp_T_break) {
        //std::cout << "[MAPPING] deltaR:" << deltaR << " < " << icp_R_break << " && deltaT: " << deltaT << " < " << icp_T_break << std::endl;
        //            break;
        //        }
    }
    //    if (deltaR > icp_R_break || deltaT > icp_T_break) {
    //        std::cout << "[MAPPING] Out of bound deltaR: " << deltaR << " , deltaT: " << deltaT << std::endl;
    //    }


    //testGetPoint2PlaneDistance();
    //testGetPlaneJacobi();
    //testCovariance();
    //testLVM();
}

void ICP::associate()
{
    // Transform input point cloud according to its current estimate
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ> ());
    Eigen::Matrix4f T = getTransformationMatrix();
    //std::cout << "T:" << T << std::endl;
    pcl::transformPointCloud (*inputCloud, *transformedCloud, T);

    pointToPlaneIdx.resize(0);
    surfaceNormals->points.resize(0);
    surfaceNormalsPosition->points.resize(0);
    transformedInputCloud->points.resize(0);

    for (int i = 0; i < transformedCloud->points.size() && pointToPlaneIdx.size()<20000; i+=10)
    {
        //std::cout << i;
        if (fabs(inputCloud->points[i].x > 1.2) || fabs(inputCloud->points[i].y > 1.2) || fabs(inputCloud->points[i].z > 1.2))
        {
            pcl::PointXYZ inputPointTransformed = transformedCloud->points[i];
            //if (fabs(inputPointTransformed.v) < 0.05 || fabs(inputPointTransformed.v + 1) < 0.05)
            //{
            pcl::PointXYZ normal;
            pcl::PointXYZ normalPosition;
            if (associatePointToPlane(inputPointTransformed,normal,normalPosition))
            {
                pointToPlaneIdx.push_back(i);
                surfaceNormals->points.push_back(normal);
                surfaceNormalsPosition->points.push_back(normalPosition);
                transformedInputCloud->points.push_back(inputPointTransformed);
                //std::cout << "pointToPlaneIdx.size=" << pointToPlaneIdx.size() << std::endl;
                //std::cout << "associated inputPointTransformed=" << inputPointTransformed << " with normalPosition=" << normalPosition << std::endl;
            }
            //}
            //else
            //{
            //processCorner();
            //}
        }
    }
    std::cout << "associated pointToPlaneIdx.size()=" << pointToPlaneIdx.size() << ", surfaceNormals=" << surfaceNormals->points.size() << std::endl;
}

template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon =
        std::numeric_limits<double>::epsilon())
{
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU |
                                          Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows())
            *svd.singularValues().array().abs()(0);
    return svd.matrixV() *  (svd.singularValues().array().abs() >
                             tolerance).select(svd.singularValues().array().inverse(),
                                               0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

void ICP::nonlinearLVM()
{
    if (!(pointToPlaneIdx.size() == surfaceNormals->points.size() && pointToPlaneIdx.size() == surfaceNormalsPosition->points.size()))
    {
        std::cout << "[ERROR]: input cloud size is different " << std::endl;
        return;
    }

    int amount = pointToPlaneIdx.size();
    if (amount < 5)
        return;

    Eigen::MatrixXf J(amount,6);
    Eigen::MatrixXf d(amount,1);

    float sum_d=0;
    for (int i=0; i < amount; i++)
    {

        Eigen::MatrixXf J_temp(1,6);
        J_temp = getPlaneJacobi(transformation, surfaceNormals->points[i], surfaceNormalsPosition->points[i], inputCloud->points[pointToPlaneIdx[i]]);
        J(i,0) = J_temp(0);
        J(i,1) = J_temp(1);
        J(i,2) = J_temp(2);
        J(i,3) = J_temp(3);
        J(i,4) = J_temp(4);
        J(i,5) = J_temp(5);
        //std::cout << "J_temp=" << J_temp << std::endl;
        d(i,0) = getPoint2PlaneDistance(surfaceNormals->points[i], surfaceNormalsPosition->points[i], transformedInputCloud->points[i]);
        d(i,0) = d(i,0) * d(i,0);
        sum_d += d(i,0);
    }

    //std::cout << "J:" << J << std::endl;

    std::cout << "sum_d=" << sum_d << std::endl;


    Eigen::MatrixXf JtJ(amount,amount);
    JtJ = J.transpose() * J;
    //std::cout << "JtJ=" << JtJ << std::endl;

    float lambda = 0.001f;
    Eigen::MatrixXf inv(amount,amount);
    Eigen::MatrixXf JtJ_diag(6,6);
    Eigen::MatrixXf JtJ_diag_vec;
    JtJ_diag_vec = JtJ.diagonal();
    JtJ_diag << JtJ_diag_vec(0,0), 0, 0, 0, 0, 0,
            0, JtJ_diag_vec(1,0), 0, 0, 0, 0,
            0, 0, JtJ_diag_vec(2,0), 0, 0, 0,
            0, 0, 0, JtJ_diag_vec(3,0), 0, 0,
            0, 0, 0, 0, JtJ_diag_vec(4,0), 0,
            0, 0, 0, 0, 0, JtJ_diag_vec(5,0);
    inv = JtJ + lambda * JtJ_diag;
    //std::cout << "JtJ.diagonal()=" << JtJ.diagonal() << std::endl;
    //std::cout << "JtJ_diag=" << JtJ_diag << std::endl;
    inv = pseudoInverse(inv);
    //std::cout << "inv" << inv << std::endl;

    Eigen::MatrixXf inc(6,1);
    inc = inv * J.transpose();
    inc = inc * d;
    //std::cout << "inc =" << inc << std::endl;
    //up = pinv(J'*J+lambda*diag(diag(J'*J))) * J' * d.^2;

    for (int i=0;i<6;i++)
        transformation[i] = transformation[i] - inc(i,0);


}

Eigen::MatrixXf ICP::getPlaneJacobi(float * theta, pcl::PointXYZ normal, pcl::PointXYZ normalPosition, pcl::PointXYZ point)
{
    float d_2_x = normal.x * normal.x;
    float d_2_y = normal.y * normal.y;
    float d_2_z = normal.z * normal.z;
    float n = d_2_x + d_2_y + d_2_z;
    Eigen::Matrix3f R;
    R = getRotationMatrix(theta[3],theta[4], theta[5]);
    Eigen::Vector3f t;
    t << theta[0], theta[1], theta[2];

    Eigen::Vector3f point_k1(point.x, point.y, point.z);
    Eigen::Vector3f point_k(normalPosition.x, normalPosition.y, normalPosition.z);
    Eigen::Vector3f a = R*point_k1+t-point_k;


    Eigen::Vector3f a_x;
    a_x = a_x_theta(theta[3],theta[4],theta[5],point_k1);
    Eigen::MatrixXf temp_x(1, 6);
    temp_x << 1, 0, 0, a_x(0), a_x(1), a_x(2);
    temp_x *= 2 * a(0) * d_2_x;

    Eigen::Vector3f a_y;
    a_y = a_y_theta(theta[3],theta[4],theta[5],point_k1);
    Eigen::MatrixXf temp_y(1, 6);
    temp_y << 0, 1, 0, a_y(0), a_y(1), a_y(2);
    temp_y *= 2 * a(1) * d_2_y;

    Eigen::Vector3f a_z;
    a_z = a_z_theta(theta[3],theta[4],theta[5],point_k1);
    Eigen::MatrixXf temp_z(1, 6);
    temp_z << 0, 0, 1, a_z(0), a_z(1), a_z(2);
    temp_z *= 2 * a(2) * d_2_z;

    return (temp_x + temp_y + temp_z) / n;
}

float ICP::getPoint2PlaneDistance(pcl::PointXYZ normal, pcl::PointXYZ normalPosition, pcl::PointXYZ point)
{
    Eigen::Vector3f temp;
    temp(0) = point.x - normalPosition.x;
    temp(1) = point.y - normalPosition.y;
    temp(2) = point.z - normalPosition.z;
    Eigen::Vector3f eigNormal(normal.x,normal.y,normal.z);
    float norm = eigNormal.norm();
    float t;
    t  = temp.transpose() * eigNormal;
    return t / norm;
}
