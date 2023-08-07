//
// Created by ty on 20-8-8.
//


#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/common/angles.h>


using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

/**
 * 
 */
int main(int argc, char *argv[]) {

    // 把room_scan1.pcd作为target目标点云
    PointCloudT::Ptr target_cloud(new PointCloudT);

    if (pcl::io::loadPCDFile("./data/room_scan1.pcd", *target_cloud) == -1) {
        PCL_ERROR("文件不存在： ./data/room_scan1.pcd\n");
        return -1;
    }

    PCL_INFO("文件 %s 加载成功, 共有点个数： %d\n", "./data/room_scan1.pcd", target_cloud->points.size());


    PointCloudT::Ptr input_cloud(new PointCloudT);
    if (pcl::io::loadPCDFile("./data/room_scan2.pcd", *input_cloud) == -1) {
        PCL_ERROR("文件不存在： ./data/room_scan2.pcd\n");
        return -1;
    }

    PCL_INFO("文件 %s 加载成功, 共有点个数： %d\n", "./data/room_scan2.pcd", input_cloud->points.size());

    PointCloudT::Ptr filtered_cloud(new PointCloudT);
    // 对输入点云进行降采样，减少点个数提高匹配速度
    pcl::ApproximateVoxelGrid<PointT> a_voxel;
    a_voxel.setLeafSize(0.2, 0.2, 0.2);
    a_voxel.setInputCloud(input_cloud);
    a_voxel.filter(*filtered_cloud);

    PCL_INFO("降采样后的点云个数量：%d \n", filtered_cloud->points.size());

    // 使用NDT对象初始化pdfs（概率密度函数块）
    pcl::NormalDistributionsTransform<PointT, PointT> ndt;
    // 设置最小的变换差异阈值，当对应点的差异小于该阈值时，停止迭代
    ndt.setTransformationEpsilon(0.01);
    // 设置线性搜索的步长
    ndt.setStepSize(0.1);
    // 设置每个cell单元格的大小.1m立方体 （会在内部计算点的协方差矩阵）
    ndt.setResolution(1.0);
    // 设置最大的迭代次数
    ndt.setMaximumIterations(35);
    // 设置输入点云, 将要用来旋转变换的点云 ---------------------------------in
    ndt.setInputSource(filtered_cloud);
    // 设置目标点云 -----------------------------------------------------in
    ndt.setInputTarget(target_cloud);

    // 设置一些初始化评估位姿
    Eigen::AngleAxisf init_rotation(pcl::deg2rad(40.0f) ,Eigen::Vector3f::UnitZ()); // 绕Z轴旋转弧度
    Eigen::Translation3f init_translation(1.79387, 0, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

    // 执行配准，准备容器（配准后原点云变换后的点云）
    PointCloudT::Ptr transformed_cloud(new PointCloudT);

    // ------------------------------------------------------------------!
    ndt.align(*transformed_cloud, init_guess);

    std::cout << "结果是否收敛：" << ndt.hasConverged()
              << " 匹配分数：" << ndt.getFitnessScore() << std::endl;
    std::cout << "变换矩阵：\n" << ndt.getFinalTransformation() << std::endl;

    // 对输入点云，执行变换， 显示output_cloud
    PointCloudT::Ptr output_cloud(new PointCloudT);
    pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());

    // Initializing point cloud visualizer
    pcl::visualization::PCLVisualizer::Ptr
            viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_final->setBackgroundColor(0, 0, 0);

    // Coloring and visualizing target cloud (red). 添加目标点云，设置为红色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            target_color(target_cloud, 255, 0, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");

    // Coloring and visualizing transformed input cloud (green). 添加变换后的输入点云，设置为绿色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            output_color(output_cloud, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output_cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "output_cloud");

    // Starting visualizer
    viewer_final->addCoordinateSystem(1.0, "global");
    viewer_final->initCameraParameters();

    // Wait until visualizer window is closed.
    while (!viewer_final->wasStopped()) {
        viewer_final->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }


}