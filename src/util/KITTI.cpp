#include "KITTI.h"

std::string &ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
}
std::string &rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    return s;
}
std::string &trim(std::string &s) {
    return ltrim(rtrim(s));
}

KITTI::KITTI()
{
    path_to_image_0 = "/home/sebastian/Dropbox/KITTI/sequences/00/image_0/";
    path_to_image_2 = "/home/sebastian/Dropbox/KITTI/sequences/00/image_2/";
    path_to_velo = "/home/sebastian/Dropbox/KITTI/velo/00/";
    pathPoses = "/home/sebastian/Dropbox/KITTI/poses";

    int num = 1150000;
    getFiles(path_to_velo, velo_files);
}

void KITTI::writeResult(std::vector<Eigen::Matrix4d> Ts)
{
    std::stringstream filename;
    filename << "03" << ".txt";
    std::string file = filename.str();

    std::ofstream myfile;
    myfile.open (file);

    for (size_t i=0;i<Ts.size();i++)
    {
        Eigen::Matrix4d T = Ts[i];
        myfile << T(0,0) << " " << T(0,1) << " " << T(0,2) << " " << T(0,3) << " "
               << T(1,0) << " " << T(1,1) << " " << T(1,2) << " " << T(1,3) << " "
               << T(2,0) << " " << T(2,1) << " " << T(2,2) << " " << T(2,3)
               << "\n";
    }
    myfile.close();
}

int KITTI::getFiles(std::string source, std::vector<std::string> &files)
{
    if(getdir(source, files) >= 0)
    {
        printf("found %d image files in folder %s!\n", (int)files.size(), source.c_str());
    }
    else if(getFile(source, files) >= 0)
    {
        printf("found %d image files in file %s!\n", (int)files.size(), source.c_str());
    }
    else
    {
        printf("could not load file list! wrong path / file? %s\n", source.c_str());
    }

    return static_cast<int>(files.size());
}

int KITTI::getVel(std::vector<std::string> &files, std::vector<std::vector<veloPoint>> &points, int num, int start)
{
    for (size_t j=start; j<files.size() && j<num;j++)
    {
        // allocate 4 MB buffer (only ~130*4*4 KB are needed)
        int32_t num = 1000000;
        float *data = (float*)malloc(num*sizeof(float));

        // pointers
        float *px = data+0;
        float *py = data+1;
        float *pz = data+2;
        float *pr = data+3;

        // load point cloud
        FILE *stream;
        stream = fopen (files[j].c_str(),"rb");
        num = fread(data,sizeof(float),num,stream)/4;
        std::vector<veloPoint> temp;
        for (int32_t i=0; i<num; i++) {
            //std::cout << "i=" << i << std::endl;
            //point_cloud.points.push_back(tPoint(*px,*py,*pz,*pr));
            veloPoint point;
            point.x = *px;
            point.y = *py;
            point.z = *pz;
            point.i = *pr;
            temp.push_back(point);
            px+=4; py+=4; pz+=4; pr+=4;
        }
        std::cout << "velopoints size" << temp.size() << std::endl;
        points.push_back(temp);
        fclose(stream);
    }
    return points.size();
}

int KITTI::getOneVel(std::vector<veloPoint> &points, int j)
{
    // allocate 4 MB buffer (only ~130*4*4 KB are needed)
    int32_t num = 1000000;
    float *data = (float*)malloc(num*sizeof(float));

    // pointers
    float *px = data+0;
    float *py = data+1;
    float *pz = data+2;
    float *pr = data+3;

    // load point cloud
    FILE *stream;
    stream = fopen (velo_files[j].c_str(),"rb");
    num = fread(data,sizeof(float),num,stream)/4;

    for (int32_t i=0; i<num; i++) {
        veloPoint point;
        point.x = *px;
        point.y = *py;
        point.z = *pz;
        point.i = *pr;
        points.push_back(point);
        px+=4; py+=4; pz+=4; pr+=4;
    }
    fclose(stream);
    return points.size();
}

int KITTI::getdir (std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL)
    {
        return -1;
    }

    while ((dirp = readdir(dp)) != NULL) {
        std::string name = std::string(dirp->d_name);

        if(name != "." && name != "..")
            files.push_back(name);
    }
    closedir(dp);


    std::sort(files.begin(), files.end());

    if(dir.at( dir.length() - 1 ) != '/') dir = dir+"/";
    for(unsigned int i=0;i<files.size();i++)
    {
        if(files[i].at(0) != '/')
            files[i] = dir + files[i];
    }

    return files.size();
}

int KITTI::getFile(std::string source, std::vector<std::string> &files)
{
    std::ifstream f(source.c_str());

    if(f.good() && f.is_open())
    {
        while(!f.eof())
        {
            std::string l;
            std::getline(f,l);

            l = trim(l);

            if(l == "" || l[0] == '#')
                continue;

            files.push_back(l);
        }

        f.close();

        size_t sp = source.find_last_of('/');
        std::string prefix;
        if(sp == std::string::npos)
            prefix = "";
        else
            prefix = source.substr(0,sp);

        for(unsigned int i=0;i<files.size();i++)
        {
            if(files[i].at(0) != '/')
                files[i] = prefix + "/" + files[i];
        }

        return (int)files.size();
    }
    else
    {
        f.close();
        return -1;
    }

}

void KITTI::getPointCloud2(sensor_msgs::PointCloud2 & outPC, int i)
{
    std::vector<veloPoint> velpoints;
    getOneVel(velpoints,i);
    createPointCloud2(outPC,velpoints);
}

void KITTI::getPointCloud(pcl::PointCloud<pcl::PointXYZ> & msg, int i)
{
    getVeloPC(msg,velpoints[i]);
    pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>);
    *source = msg;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (source);
    sor.setLeafSize (0.4f, 0.4f, 0.4f);
    sor.filter (*newCloud);
    msg = *newCloud;
}

void KITTI::createPointCloud2(sensor_msgs::PointCloud2 & outPC, std::vector<veloPoint> & veloPoints)
{
    pcl::PointCloud<pcl::PointXYZ> msg;
    getVeloPC(msg,veloPoints);
    pcl::toROSMsg(msg,outPC);
    msg.clear();
}

void KITTI::getVeloPC(pcl::PointCloud<pcl::PointXYZ> & msg, std::vector<veloPoint> & veloPoints)
{
    msg.is_dense = true;
    msg.header.seq = 1;
    msg.header.stamp = 1;
    msg.header.frame_id = "some_tf_frame";
    msg.height = 1;
    msg.resize(sizeof(pcl::PointXYZ)*veloPoints.size());
    for (int i=0; i<veloPoints.size(); i++)
    {
        //pcl::PointXYZI temp(10.0);
        pcl::PointXYZ temp;
        temp.x = veloPoints[i].x;
        temp.y = veloPoints[i].y;
        temp.z = veloPoints[i].z;
        //temp.intensity = veloPoints[i].i;

        msg.points.push_back (temp);
    }

    msg.width = msg.points.size();
    Eigen::Matrix4d T = getVelo_to_cam_T();
    pcl::transformPointCloud (msg, msg, T);
}



Eigen::Matrix3d KITTI::getVelo_to_cam_R()
{
    Eigen::Matrix3d M;
    M << 7.967514e-03, -9.999679e-01, -8.462264e-04, -2.771053e-03, 8.241710e-04, -9.999958e-01, 9.999644e-01, 7.969825e-03, -2.764397e-03;
    return M;
}

Eigen::Vector3d KITTI::getVelo_to_cam_t(){
    Eigen::Vector3d v;
    v << -1.377769e-02, -5.542117e-02, -2.918589e-01;
    return v;
}

Eigen::Matrix4d KITTI::getVelo_to_cam_T()
{
    Eigen::Matrix3d velo_to_cam_R = getVelo_to_cam_R();
    Eigen::Vector3d velo_to_cam_t = getVelo_to_cam_t();
    Eigen::Matrix4d T;
    T << velo_to_cam_R(0,0), velo_to_cam_R(0,1), velo_to_cam_R(0,2), velo_to_cam_t(0),
            velo_to_cam_R(1,0), velo_to_cam_R(1,1), velo_to_cam_R(1,2), velo_to_cam_t(1),
            velo_to_cam_R(2,0), velo_to_cam_R(2,1), velo_to_cam_R(2,2), velo_to_cam_t(2),
            0 , 0 , 0 ,1;
    return T;
}

void KITTI::getGtCameraPosesAsNavMsg(std::vector<nav_msgs::Odometry> &out)
{
    std::vector<Eigen::Matrix3d> Rs;
    std::vector<Eigen::Vector3d> ts;
    getGtCameraPoses(Rs,ts);

    for (int i=0;i<ts.size();i++)
    {
        nav_msgs::Odometry temp;
        temp.header.frame_id = "/camera_init_2";
        temp.child_frame_id = "/camera";

        Eigen::Quaterniond q(Rs[i]);

        temp.header.stamp = ros::Time().now();
        temp.pose.pose.orientation.x = q.x();
        temp.pose.pose.orientation.y = q.y();
        temp.pose.pose.orientation.z = q.z();
        temp.pose.pose.orientation.w = q.w();
        temp.pose.pose.position.x = ts[i][0];
        temp.pose.pose.position.y = ts[i][1];
        temp.pose.pose.position.z = -ts[i][2];

        out.push_back(temp);
    }
}

void KITTI::getGtCameraPoses(std::vector<Eigen::Matrix4d> &Ts)
{
    std::vector<Eigen::Matrix3d> Rs;
    std::vector<Eigen::Vector3d> ts;
    getGtCameraPoses(Rs,ts);
    for (int i=0;i<Rs.size();i++)
    {
        Eigen::Matrix3d velo_to_cam_R = Rs[i];
        Eigen::Vector3d velo_to_cam_t = ts[i];
        Eigen::Matrix4d T;
        T << velo_to_cam_R(0,0), velo_to_cam_R(0,1), velo_to_cam_R(0,2), velo_to_cam_t(0),
                velo_to_cam_R(1,0), velo_to_cam_R(1,1), velo_to_cam_R(1,2), velo_to_cam_t(1),
                velo_to_cam_R(2,0), velo_to_cam_R(2,1), velo_to_cam_R(2,2), velo_to_cam_t(2),
                0 , 0 , 0 ,1;
        Ts.push_back(T);
    }
}


int KITTI::getGtCameraPoses(std::vector<Eigen::Matrix3d> &Rs, std::vector<Eigen::Vector3d> &ts)
{

    std::string  filepath = pathPoses +"/03.txt";
    std::cout << "Try to get poses from :" << filepath << std::endl;
    std::ifstream f(filepath.c_str());

    if(f.good() && f.is_open())
    {
        while(!f.eof())
        {
            std::string l;
            std::getline(f,l);

            std::istringstream in(l);

            double r11, r12, r13, r21, r22, r23, r31, r32, r33, t1, t2, t3;
            in.precision(20);
            in >> r11 >> r12 >> r13 >> t1 >> r21 >> r22 >> r23 >> t2 >> r31 >> r32 >> r33 >> t3;

            Eigen::Matrix3d R;
            R << r11 , r12 , r13 , r21, r22, r23, r31, r32, r33;
            //std::cout << "R=" << R << std::endl;
            Rs.push_back(R);


            Eigen::Vector3d t;
            t(0) = t1;
            t(1) = t2;
            t(2) = t3;
            //std::cout << "t="  << t << std::endl;
            ts.push_back(t);


        }
        std::cout << "Set Camera poses ... OK" << std::endl;
        f.close();
    }
    else
    {
        std::cout << "could not load poses file from :" << filepath << std::endl;
        f.close();
        return -1;
    }
}
