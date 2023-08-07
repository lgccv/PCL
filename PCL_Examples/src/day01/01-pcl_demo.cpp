//
// Created by ty on 20-8-2.
/***
 * 加载一个点云文件，并展示出来
 */


#include <iostream>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

/**
 * 
 */
int main(int argc, char *argv[]) {

    // 创建点云的智能指针, 盛放点云
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGBA>);

    // 从pcd文件加载点云到cloud
    pcl::io::loadPCDFile("./data/pcl_logo.pcd", *cloud);

    // 展示点云，创建CloudViewer
    pcl::visualization::CloudViewer viewer("cloud viewer");

    // 此代码会阻塞直到渲染完毕
    viewer.showCloud(cloud);

    // 死循环判断窗口是否退出, 用户按了q
    while (!viewer.wasStopped()) {
        // 不作任何处理，可以在这里更新点云内容
    }
    return 0;
}