//
// Created by ty on 20-8-2.
//


#include <iostream>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

using namespace std;

/**
 * 点云的反序列化
 */
int main(int argc, char *argv[]) {

    // 加载点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile("./data/bunny.pcd", *cloud) == -1) {
        PCL_ERROR("不能加载点云./data/bunny.pcd");
        return -1;
    }

    std::cout << "加载成功， 共有点数：" << cloud->width * cloud->height << std::endl;

    unsigned long size = cloud->points.size();
    for (int i = 0; i < size; ++i) {
        std::cout <<
             "x:" << cloud->points[i].x <<
            " y:" << cloud->points[i].y <<
            " z:" << cloud->points[i].z <<
            std::endl;
    }

    return 0;
}