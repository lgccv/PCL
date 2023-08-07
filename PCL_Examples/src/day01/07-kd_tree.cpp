//
// Created by ty on 20-8-2.
//


#include <iostream>
#include <pcl/io/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

/**
 * 
 */
int main(int argc, char *argv[]) {

    // 系统时间初始化随机种子
    srand(time(NULL));

    // 在 0 -> 1024.0f 随机生成1000个点
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    cloud->width = 1000;
    cloud->height = 1;
    cloud->is_dense = false; // 非密集
    cloud->points.resize(cloud->width * cloud->height);
    unsigned long size = cloud->points.size();
    for (int i = 0; i < size; ++i) {
        cloud->points[i].x = 1024.0f * (rand() / (RAND_MAX + 1.0f));
        cloud->points[i].y = 1024.0f * (rand() / (RAND_MAX + 1.0f));
        cloud->points[i].z = 1024.0f * (rand() / (RAND_MAX + 1.0f));
    }

    // 创建KdTree实现类对象 KdTreeFLANN (Fast Library Approximate Nearest Neighbor)
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//    pcl::KdTree<pcl::PointXYZ> kdtree;
    // 设置输入点云
    kdtree.setInputCloud(cloud);

    // 随机选一个点作为搜索点
    pcl::PointXYZ searchPoint;
    searchPoint.x = 1024.0f * (rand() / (RAND_MAX + 1.0f));
    searchPoint.y = 1024.0f * (rand() / (RAND_MAX + 1.0f));
    searchPoint.z = 1024.0f * (rand() / (RAND_MAX + 1.0f));


    // 方式一：指定搜索最近的K个邻居 K nearest neighbor search (KNN)
    int K = 10;
    // 准备好输出结果列表 - 邻居索引列表
    vector<int> pointIdKnnSearchIndices(K);
    // 准备好输出结果列表 - 邻居距离平方的列表
    vector<float> pointKnnSearchDistance(K);

    std::cout << "K nearest neighbor search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with K=" << K << std::endl;

    // 执行knn -> k邻域搜索， 返回值邻居的个数
    if (kdtree.nearestKSearch(searchPoint, K, pointIdKnnSearchIndices, pointKnnSearchDistance) > 0){
        unsigned long realCount = pointIdKnnSearchIndices.size();

        for (int i = 0; i < realCount; ++i) {
            int index = pointIdKnnSearchIndices[i];
            float dis = pointKnnSearchDistance[i];
            std::cout << "("
                << cloud->points[index].x << ", "
                << cloud->points[index].y << ", "
                << cloud->points[index].z << " "
                << " 距离(" << sqrt(dis) << " )"
            << std::endl;
        }
    }

    // 方式二：通过指定半径搜索邻居

    float radius = 256.0f * rand() / (RAND_MAX + 1.0f);
    // 准备好输出结果列表 - 邻居索引列表
    vector<int> pointIdRadiusSearchIndices;
    // 准备好输出结果列表 - 邻居距离平方的列表
    vector<float> pointRadiusSearchDistance;

    std::cout << "Nearest neighbor search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with Radius=" << radius << std::endl;

    if (kdtree.radiusSearch(searchPoint, radius, pointIdKnnSearchIndices, pointRadiusSearchDistance) > 0) {
        unsigned long realCount = pointIdKnnSearchIndices.size();
        for (int i = 0; i < realCount; ++i) {
            int index = pointIdKnnSearchIndices[i];
            float dis = pointRadiusSearchDistance[i];
            std::cout << "("
                      << cloud->points[index].x << ", "
                      << cloud->points[index].y << ", "
                      << cloud->points[index].z << " "
                      << " 距离(" << sqrt(dis) << " )"
                      << std::endl;
        }

    }

    pcl::visualization::PCLVisualizer viewer("cloud_viewer");
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    pcl::PointXYZ orgPoint(0, 0, 0);
    viewer.addLine(orgPoint, searchPoint);
    viewer.addSphere(searchPoint, radius);

    viewer.addCoordinateSystem(200.0f);
    while (!viewer.wasStopped()) {
        // 每次循环手动调用渲染函数
        viewer.spinOnce();
    }
}