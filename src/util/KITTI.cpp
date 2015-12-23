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

    int num = 15;
    getFiles(path_to_velo, velo_files);
    getVel(velo_files,velpoints,num);
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

int KITTI::getVel(std::vector<std::string> &files, std::vector<std::vector<veloPoint>> &points, int num)
{
    for (size_t j=0; j<files.size() && j<num;j++)
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
        points.push_back(temp);
        fclose(stream);
    }
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
