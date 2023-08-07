//
// Created by ty on 20-8-2.
/**
 * 加载点云
 * 进行旋转
 * 进行平移
 */

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>

using namespace std;

/**
 * 
 */
int main(int argc, char *argv[]) {


    // 加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("./data/bunny.pcd", *cloud);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D_Viewer"));
    viewer->setBackgroundColor(0.05, 0.05, 0.05, 0);
    // 给点云设置个纯色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample_cloud");

    // 方式一： 使用4X4变换矩阵 ------------------------------------------------------
    /* Reminder: how transformation matrices work :

              |-------> This column is the translation
      | 1 0 0 x |  \
      | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
      | 0 0 1 z |  /
      | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

      METHOD #1: Using a Matrix4f
      This is the "manual" method, perfect to understand but error prone !
    */
    // 对兔子进行旋转平移后，再展示
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

    // 绕z轴顺时针旋转45°
    float theta = - M_PI / 4;
    transform_1(0, 0) = cos(theta);
    transform_1(0, 1) =-sin(theta);
    transform_1(1, 0) = sin(theta);
    transform_1(1, 1) = cos(theta);

    // 在x轴方向上平移1.5m
    transform_1(0, 3) = 1.5;

    std::cout << "transform_1:\n" << transform_1 << std::endl;

    // 方式二：使用Eigen提供的仿射变换对象 ---------------------------------------------------
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    // 执行旋转操作 z逆时针30度
    const Eigen::AngleAxis<float> &rotation = Eigen::AngleAxisf(DEG2RAD(30), Eigen::Vector3f::UnitZ());
//    const Eigen::AngleAxis<float>::Matrix3 &matrix = rotation.toRotationMatrix();
    transform_2.rotate(rotation);

    // 执行x平移操作
    transform_2.translation() << 1.0, 0, 0;
    std::cout << "transform_2:\n" << transform_2.matrix() << std::endl;


    // 执行变换并将之输出到transformed_cloud中
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform_2);

    // 可视化
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud, 0,255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(transformed_cloud, single_color2, "sample_cloud2");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample_cloud2");

    // 添加一个坐标系
    viewer->addCoordinateSystem(1.0);

    while (!viewer->wasStopped()) {
        // 每次循环手动调用渲染函数
        viewer->spinOnce();
    }
}