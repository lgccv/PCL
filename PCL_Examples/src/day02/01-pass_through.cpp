//
// Created by ty on 20-8-4.
// 直通滤波
//


#include <iostream>
#include <pcl/io/io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

/**
 * 随机生成5个点，进行直通滤波
 */
int main(int argc, char *argv[]) {

    srand(time(nullptr));
    // 0 -> 1024 | -1.0 -> 1.0
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filterd(new pcl::PointCloud<pcl::PointXYZ>);

    cloud->width = 5;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    std::cout << "rand(): " << rand() << std::endl; // 0 -> 2^31 - 1  * (2 ^ 10)
    std::cout << "1024 * rand(): " << 1024 * rand() << std::endl; // 0 -> 2^31 - 1  * (2 ^ 10)
    std::cout << "(RAND_MAX + 1.0): " << (RAND_MAX + 1.0)<< std::endl;
//    00111111000000010101001100000011 0000000000

    std::cout << "直通滤波之前的5个点" << std::endl;
    for (int i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0);
        cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0);
        cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0);

        printf("(%f, %f, %f)\n", cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    }

    // 创建滤波器对象
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");   // 要过滤的字段
    pass.setFilterLimits(0.0, 1.0); // 设置保留范围
//    pass.setFilterLimitsNegative(true); // 默认false，如果true，保留Limits范围之外内容
    pass.filter(*cloud_filterd);    // 执行过滤，并把结果放到cloud_filterd

    std::cout << "过滤后留下的点------------------------" << std::endl;
    for (int i = 0; i < cloud_filterd->points.size(); ++i) {
        printf("(%f, %f, %f)\n", cloud_filterd->points[i].x, cloud_filterd->points[i].y, cloud_filterd->points[i].z);
    }

    pcl::visualization::CloudViewer viewer("cloud_viewer");
    viewer.showCloud(cloud);

    while (!viewer.wasStopped()) {

    }

    return 0;
}