//
// Created by ty on 20-8-4.
//

#include <iostream>
#include <thread>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;


/**
 * 使用方法：
 *
 * random_sample_consensus     创建包含外部点的平面
 * random_sample_consensus -f  创建包含外部点的平面，并计算平面内部点
 *
 * random_sample_consensus -s  创建包含外部点的球体
 * random_sample_consensus -sf 创建包含外部点的球体，并计算球体内部点
 */
int main(int argc, char *argv[]) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inliers_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 根据参数生成点云数据

    cloud->width = 500;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    bool isSphere = pcl::console::find_argument(argc, argv, "-s") >= 0 || pcl::console::find_argument(argc, argv, "-sf") >= 0;


    for (int i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        if (isSphere) {
            // -s 生成包含外部点球体
            if (i % 5 == 0) {
                cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
            } else if (i % 2 == 0) {
                cloud->points[i].z =  sqrt(1 - (pow(cloud->points[i].x, 2) + pow(cloud->points[i].y, 2)));
            } else {
                cloud->points[i].z = -sqrt(1 - (pow(cloud->points[i].x, 2) + pow(cloud->points[i].y, 2)));
            }

        }else {
            // 生成包含外部点平面 -1.0 -> 1.0
            if (i % 2 == 0) {
                cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
            }else {
                cloud->points[i].z = -1.0f * (cloud->points[i].x + cloud->points[i].y);
            }
        }
    }

    bool isRansac = pcl::console::find_argument(argc, argv, "-f") >= 0 || pcl::console::find_argument(argc, argv, "-sf") >= 0;

    if (isRansac) {

        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(
                new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));


        pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_s(
                new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));

        // 用于接收所有内部点的索引
        std::vector<int> inliers;

        // 根据模型迭代拟合出平面模型参数，并获取符合其模型参数的内部点
        pcl::SampleConsensusModel<pcl::PointXYZ>::Ptr model_arg = model_p;
        if (isSphere) {
            // 根据模型迭代拟合出球体模型参数，并获取符合其模型参数的内部点
            model_arg = model_s;

        }
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_arg);
        // 阈值 1cm
        ransac.setDistanceThreshold(0.01f);
        // 执行计算
        ransac.computeModel();
        // 获取内点
        ransac.getInliers(inliers);


        // 根据索引列表拷贝点云
        pcl::copyPointCloud(*cloud, inliers, *inliers_cloud);
    }

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D_Viewer"));
    viewer->setBackgroundColor(0.05, 0.05, 0.05, 0);
    // 给点云设置个纯色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 255, 255);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample_cloud");

    if (isRansac) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> inliers_color(cloud, 255, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ>(inliers_cloud, inliers_color, "inliers_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "inliers_cloud");
    }

    // 添加一个坐标系
    viewer->addCoordinateSystem(0.5);

    while (!viewer->wasStopped()) {
        // 每次循环手动调用渲染函数
        viewer->spinOnce();
    }

}