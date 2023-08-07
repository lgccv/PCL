//
// Created by ty on 20-8-2.
//


#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>

using namespace std;

/**
 * 
 */
int main(int argc, char *argv[]) {

    // 加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("./data/bunny.pcd", *cloud);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_milk(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile("./data/milk_color.pcd", *cloud_milk);


    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D_Viewer"));

    viewer->setBackgroundColor(0.05, 0.05, 0.05, 0);

    // 给点云设置个纯色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample_cloud");

    viewer->addPointCloud<pcl::PointXYZRGBA>(cloud_milk, "sample_color_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample_color_cloud");

    // 添加一个坐标系
    viewer->addCoordinateSystem(0.5);

    while (!viewer->wasStopped()) {
        // 每次循环手动调用渲染函数
        viewer->spinOnce();
    }

    return 0;
}