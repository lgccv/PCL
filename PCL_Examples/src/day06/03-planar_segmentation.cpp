//
// Created by ty on 20-8-10.
//


#include <iostream>
#include <pcl/io/io.h>
#include <pcl/console/print.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;

/***
 * 生成15个无序点云，x,y为随机数，z为1.0
 * 将points中0、3、6索引位置的z值进行修改，将之作为离群值
 */
int main(int argc, char *argv[]) {


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    
    cloud->width = 15;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    
    // -0.1 -> 0.1
    for (int i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1.0;
    }

    cloud->points[0].z = 2.0;
    cloud->points[3].z = -2.0;
    cloud->points[6].z = 3.5;

    pcl::console::print_highlight("共有点个数：%d \n", cloud->points.size());

    for (int i = 0; i < cloud->points.size(); ++i) {
        pcl::PointXYZ p = cloud->points[i];
        PCL_INFO("( %f, %f, %f ) \n", p.x, p.y, p.z);
    }

    // 创建分割模型系数对象 (用于接收估算出来的模型系数) Output
    pcl::ModelCoefficients::Ptr cofficients(new pcl::ModelCoefficients);
    // 创建存储内点索引列表 Output
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // 创建基于采样一致性模型分割器
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // 是否自动优化模型系数（可选）
    seg.setOptimizeCoefficients(true);
    // 设置模型类型（必选）
    seg.setModelType(pcl::SACMODEL_PLANE);
    // 设置分割函数类型
    seg.setMethodType(pcl::SAC_RANSAC);
    // 设置距离阈值
    seg.setDistanceThreshold(0.01);
    // 设置输入点云
    seg.setInputCloud(cloud);
    // 执行分割，把分割出来的点云索引保存到inliers里，把模型系数存到cofficients
    seg.segment(*inliers, *cofficients);

    if (inliers->indices.size() == 0) {
        PCL_ERROR("在给定的点云没有找到平面模型及对应的点");
        return -1;
    }

    // 平面模型：ax+by+cz+d = 0 ， 查看SACMODEL_PLANE相关介绍
    std::cout << "模型系数ModelCoefficients："
                << cofficients->values[0] << ", "
                << cofficients->values[1] << ", "
                << cofficients->values[2] << ", "
                << cofficients->values[3] << ", "
                << std::endl;

    pcl::console::print_highlight("符合平面模型的所有内点： %d", inliers->indices.size());

    for (int i = 0; i < inliers->indices.size(); ++i) {
        int index = inliers->indices[i];

        pcl::PointXYZ &point = cloud->points[index];
        PCL_INFO("( %f, %f, %f ) \n", point.x, point.y, point.z);
    }

    return 0;

}