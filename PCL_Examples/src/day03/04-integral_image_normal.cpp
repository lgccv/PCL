//
// Created by ty on 20-8-5.
//


#include <iostream>
#include <thread>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

/**
 * 积分图估算点云法线
此方法进行法线估计只适用于有序点云,对于无序点云就只能采用其他方法。
 */
int main(int argc, char *argv[]) {


    char *arg = argv[1];
    if (!argv[1])  arg = "./data/table_scene_mug_stereo_textured.pcd";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(arg, *cloud);

    // 创建法线集合
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    // 创建一个积分图法线评估对象
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> iine;

    // 设置法线估算方式
    /**
        enum NormalEstimationMethod

        COVARIANCE_MATRIX : 模式创建9个积分图像，以根据其局部邻域的协方差矩阵为特定点计算法线
        AVERAGE_3D_GRADIENT: 模式创建6个积分图像以计算水平和垂直3D渐变的平滑版本，并使用这两个渐变之间的叉积计算法线。
        AVERAGE_DEPTH_CHANGE: 模式仅创建单个积分图像，并根据平均深度变化计算法线。
     */
    iine.setNormalEstimationMethod(iine.AVERAGE_3D_GRADIENT);

    // 设置最大深度变换因子
    iine.setMaxDepthChangeFactor(0.02f);

    iine.setNormalSmoothingSize(10.0f);

    iine.setInputCloud(cloud);

    iine.compute(*normals);

    pcl::visualization::PCLVisualizer viewer("PCLViewer");
    viewer.addPointCloud(cloud, "cloud");
    // 每几个点绘制一个法向量
    int level = 10;
    float scale = 0.02;
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, level, scale, "normal_cloud");

    viewer.addCoordinateSystem(0.01);

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
        std::this_thread::sleep_for(100ms);
    }


}