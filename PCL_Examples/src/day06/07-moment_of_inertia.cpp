//
// Created by ty on 20-8-10.
//


#include <iostream>
#include <vector>
#include <thread>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
using namespace std;

/**
 * 3D物体包容盒
 *
 * AABB
 * OBB
 *
 */
int main(int argc, char *argv[]) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile(argv[1], *cloud) == -1) {
        std::cerr << "加载失败" << std::endl;
        return -1;
    }

    // 创建惯性矩估算对象，设置输入点云并计算
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> extractor;
    extractor.setInputCloud(cloud);
    extractor.compute();

    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
    extractor.getMomentOfInertia(moment_of_inertia);
    extractor.getEccentricity(eccentricity);

    pcl::PointXYZ min_point_AABB, max_point_AABB;
    // 获取并展示AABB包容盒子
    extractor.getAABB(min_point_AABB, max_point_AABB);

    pcl::PointXYZ min_point_OBB, max_point_OBB, position_OBB;
    Eigen::Matrix3f rotation_matrix_OBB;
    // 获取并展示OBB包容盒子
    extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotation_matrix_OBB);

    // 获取特征值、特征向量、质心
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    extractor.getEigenValues(major_value, middle_value, minor_value);
    extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
    extractor.getMassCenter(mass_center);

    // -------------------------------------------------
    pcl::visualization::PCLVisualizer viewer("viewer");
    viewer.addCoordinateSystem(1.0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "src_cloud");
    // 添加AABB包容盒
    viewer.addCube(
            min_point_AABB.x, max_point_AABB.x,
            min_point_AABB.y, max_point_AABB.y,
            min_point_AABB.z, max_point_AABB.z,
            1.0, 1.0, 0.0, "AABB");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");


    Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Quaternionf quat(rotation_matrix_OBB);
    // 添加OBB包容盒
    viewer.addCube(position, quat,
                   max_point_OBB.x - min_point_OBB.x,
                   max_point_OBB.y - min_point_OBB.y,
                   max_point_OBB.z - min_point_OBB.z,
                   "OBB");

    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");

    // 绘制坐标系
    pcl::PointXYZ center(mass_center[0], mass_center[1], mass_center[2]);
    major_vector += mass_center;
    middle_vector += mass_center;
    minor_vector += mass_center;
    // 生成x轴末端点
    pcl::PointXYZ x_axis(major_vector[0], major_vector[1], major_vector[2]);
    // 生成y轴末端点
    pcl::PointXYZ y_axis(middle_vector[0], middle_vector[1], middle_vector[2]);
    // 生成z轴末端点
    pcl::PointXYZ z_axis(minor_vector[0], minor_vector[1], minor_vector[2]);

    viewer.addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
    viewer.addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
    viewer.addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");


    while (!viewer.wasStopped()) {
        viewer.spinOnce();
        std::this_thread::sleep_for(100ms);
    }

}