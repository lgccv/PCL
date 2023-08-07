//
// Created by ty on 20-8-10.
//

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

/**
 * 重采样点云表面
 */
int main(int argc, char *argv[]) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("./data/target.pcd", *cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);

    // 创建移动最小二乘法对象
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals(true);
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(2); // 多项式的阶数
    mls.setSearchMethod(kdtree);
    mls.setSearchRadius(.03); // 邻域半径

    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);
    // 执行表面重建
    mls.process(*mls_points);

    pcl::visualization::CloudViewer viewer("viewer");

    viewer.showCloud(cloud);

    while (!viewer.wasStopped()) {
        //
    }

    pcl::io::savePCDFileASCII("target_mls.pcd", *mls_points);



}