//
// Created by ty on 20-8-5.
//


#include <iostream>
#include <pcl/io/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
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
        arg = "./data/bunny.pcd";
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(arg, *cloud);

    // 创建法线集合
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    // 计算点云法向量
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;

    // 设置输入点云
    normalEstimation.setInputCloud(cloud);

    // 设置搜索半径, 对于每个点，通过其周围半径3cm以内的邻居计算法线
    normalEstimation.setRadiusSearch(0.03);

    // 准备一个indices索引集合，为了简单起见，我们直接使用点云的前10%的点
    std::vector<int> indices (std::floor (cloud->points.size () / 10));

    for (std::size_t i = 0; i < indices.size (); ++i) indices[i] = i;
    pcl::IndicesPtr indicesPtr(new std::vector<int>(indices));

    normalEstimation.setIndices(indicesPtr);
//    normalEstimation.setSearchSurface()

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>());
    // 设置搜索方法
    normalEstimation.setSearchMethod(kdtree);

    // 估算法线
    normalEstimation.compute(*normals);

    pcl::visualization::PCLVisualizer viewer("PCLViewer");

    viewer.addPointCloud(cloud, "cloud");


    pcl::PointCloud<pcl::PointXYZ>::Ptr indicesCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, indices, *indicesCloud);
    // 这里的点云数量必须和法线数量一致 ------------------------------------ !
    // 每几个点绘制一个法向量
    int level = 1;
    float scale = 0.02;
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(indicesCloud, normals, level, scale, "normal_cloud");

    viewer.addCoordinateSystem(0.01);

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
        std::this_thread::sleep_for(100ms);
    }

    return 0;

}