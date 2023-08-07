//
// Created by ty on 20-8-5.
//


#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

/**
 * 加载点云
 * 计算法向量
 * 计算PFH
 */
int main(int argc, char *argv[]) {

    // 加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("./data/bunny.pcd", *cloud);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new  pcl::PointCloud<pcl::Normal>);

    // 估算法向量
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);
    // 法向量邻域参考范围
    normalEstimation.setRadiusSearch(0.03);
    // 设置邻域查找方式
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdtree);
    // 得到法向量
    normalEstimation.compute(*normals);

    // 创建PFH估算对象，把数据集和法线作为参数
    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
    // 设置输入数据
    pfh.setInputCloud(cloud);
    pfh.setInputNormals(normals);
    pfh.setSearchMethod(kdtree);
    // 使用半径为8cm的球体，作为搜索的邻域。必须要比用于估算法向量的半径要大！！！！！！！！
    pfh.setRadiusSearch(0.08);
    // 计算特征
    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_output(new  pcl::PointCloud<pcl::PFHSignature125>);
    pfh.compute(*pfh_output);

    // 打印输出结果
    unsigned long size = pfh_output->points.size();
    for (int i = 0; i < size; ++i) {
        pcl::PFHSignature125 &signature125 = pfh_output->points[i];
        float* hh = signature125.histogram;

        printf("%d: %f, %f, %f, %f, %f \n", i, hh[1], hh[2], hh[3], hh[4], hh[5]);
    }

    std::cout << "PFH直方图个数：" << size << std::endl;
    std::cout << "cloud点云个数：" << cloud->points.size() << std::endl;

    // 可以显示法线
    pcl::visualization::PCLVisualizer viewer("viewer");
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 1, 0.01, "normals");

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

    return 0;

}