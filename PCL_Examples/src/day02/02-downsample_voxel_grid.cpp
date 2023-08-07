//
// Created by ty on 20-8-4.
//


#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

/**
 * 
 */
int main(int argc, char *argv[]) {

    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr cloud_filterd(new pcl::PCLPointCloud2);

//    pcl::io::loadPCDFile("./data/table_scene_lms400.pcd", *cloud);
    pcl::PCDReader reader;
    reader.read("./data/table_scene_lms400.pcd", *cloud);

    std::cout << "过滤前的点个数：" << cloud->width * cloud->height << std::endl;
    std::cout << "数据类型：" << pcl::getFieldsList(*cloud)<< std::endl;

    float leafSize = 0.01f;
    // 执行降采样------------------------
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel;
    // 设置输入点云
    voxel.setInputCloud(cloud);
    // 设置体素网格的大小
    voxel.setLeafSize(leafSize, leafSize, leafSize);
    // 执行滤波, 并将结果保存到cloud_filterd
    voxel.filter(*cloud_filterd);

    std::cout << "---------------------------------------------" << std::endl;
    std::cout << "过滤后的点个数：" << cloud_filterd->width * cloud_filterd->height << std::endl;
    std::cout << "数据类型：" << pcl::getFieldsList(*cloud_filterd)<< std::endl;

    pcl::PCDWriter writer;
    writer.write("./data/table_scene_lms400_downsample.pcd", *cloud_filterd);

}