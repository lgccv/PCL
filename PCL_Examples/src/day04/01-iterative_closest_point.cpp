//
// Created by ty on 20-8-7.
//


#include <iostream>
#include <pcl/io/io.h>
#include <pcl/registration/icp.h>

using namespace std;

/**
 * 
 */
int main(int argc, char *argv[]) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);

    // 填充无序点云
    cloud_in->width = 5;
    cloud_in->height = 1;
    cloud_in->points.resize(cloud_in->width * cloud_in->height);

    std::cout << "-------------------------------------source" << std::endl;
    // -1.0 -> 1.0
    for (int i = 0; i < cloud_in->points.size(); ++i) {
        cloud_in->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_in->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_in->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
        printf("(%f, %f, %f)\n", cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z);
    }

    std::cout << "-------------------------------------target" << std::endl;
    // 将原始点云的所有点复制给目标点云
    *cloud_target = *cloud_in;

    // 执行了一个x方向上的平移，0.7m
    for (int i = 0; i < cloud_target->points.size(); ++i) {
        cloud_target->points[i].x = cloud_in->points[i].x + 0.7f;
        printf("(%f, %f, %f)\n", cloud_target->points[i].x, cloud_target->points[i].y, cloud_target->points[i].z);
    }

    // 创建ICP实例对象（IterativeClosestPoint）
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // 将cloud_in作为输入点云
    icp.setInputSource(cloud_in);
    // 将cloud_target作为目标点云
    icp.setInputTarget(cloud_target);

    pcl::PointCloud<pcl::PointXYZ> final;
    // 输出原点云经过配准变换之后的点云
    icp.align(final);

    std::cout << "结果是否收敛：" << icp.hasConverged() << std::endl; // 值==1时，表示结果收敛了（true）
    std::cout << "匹配分数（每一组对应点距离）：" << icp.getFitnessScore() << std::endl;
    const pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 &matrix = icp.getFinalTransformation();
    std::cout << "matrix:\n" << matrix << std::endl;
/**

            1           0            0          0.7
            0           1            0            0
            0           0            1            0
            0           0            0            1

 */
}