//
// Created by ty on 20-8-5.
//


#include <iostream>
#include <pcl/io/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <thread>

using namespace std;

/**
 * 评估点云法向量
 *
 * NormalEstimation
 *
 * setInputCloud
 *
 * setIndices
 *
 * setSearchSurface
 */
int main(int argc, char *argv[]) {

    char *arg = argv[1];
    if (!argv[1]) {
        arg = "./data/target.pcd";
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(arg, *cloud);

    // 创建法线集合
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    // 降采样原始点云
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.03f, 0.03f, 0.03f );
    voxel.filter(*cloud_downsampled);

    // 计算点云法向量
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    // 设置输入点云
    normalEstimation.setInputCloud(cloud_downsampled);
    // 设置搜索半径, 对于每个点，通过其周围半径3cm以内的邻居计算法线
    normalEstimation.setRadiusSearch(0.03);

//    normalEstimation.setViewPoint()

//    normalEstimation.setIndices(indicesPtr);
    normalEstimation.setSearchSurface(cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>());
    // 设置搜索方法
    normalEstimation.setSearchMethod(kdtree);

    // 估算法线
    normalEstimation.compute(*normals);

    pcl::visualization::PCLVisualizer viewer("PCLViewer");

    viewer.addPointCloud(cloud, "cloud");

    // 这里的点云数量必须和法线数量一致 ------------------------------------ !
    // 每几个点绘制一个法向量
    int level = 1;
    float scale = 0.02;
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_downsampled, normals, level, scale, "normal_cloud");

    // 所有原始点
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

    // 降采样后的点
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud_downsampled, single_color, "cloud_downsampled");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_downsampled");


    viewer.addCoordinateSystem(0.01);

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
        std::this_thread::sleep_for(100ms);
    }

    return 0;

}