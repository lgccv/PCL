//
// Created by ty on 20-8-7.

//


#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>
#include <pcl/registration/icp.h>

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

void
print4x4Matrix (const Eigen::Matrix4d & matrix)
{
    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

bool next_iteration = false;
void keyboardEventCallback(const pcl::visualization::KeyboardEvent& event, void* nothing){

    if (event.keyDown() && event.getKeySym() == "space") {
        next_iteration = true;
    }

};

/**
 * ICP
 * 每次用户按下“空格”，都会进行一次ICP迭代，并刷新查看器
 *
 * 02-interactive_icp monkey.ply 1
 *
 */
int main(int argc, char *argv[]) {

    // 创建原始点云和目标点云的容器
    PointCloudT::Ptr cloud_src(new PointCloudT);    // 原始点云
    PointCloudT::Ptr cloud_target(new PointCloudT); // 目标点云
    PointCloudT::Ptr cloud_icp(new PointCloudT);    // ICP输出点云

    // 要求用户至少有一个参数
    if (argc < 2) {
        std::cout << "使用方式：\n" << std::endl;
        std::cout << argv[0] << " file.ply 初始迭代次数" << std::endl;
        PCL_ERROR("请至少提供一个ply文件");
        return -1;
    }

    int iterations = 1;
    if (argc > 2) {
        iterations = atoi(argv[2]);
        if (iterations < 0) {
            PCL_ERROR("初始的变换次数必须要>=1");
            return -1;
        }
    }

    pcl::console::TicToc time;
    time.tic();
    if (pcl::io::loadPLYFile(argv[1], *cloud_src) < 0) {

        PCL_ERROR("加载失败：%s\n", argv[1]);
        return -1;
    }

    PCL_INFO("加载【%s】成功，共有%d个点，耗时：%f，初始迭代：%d\n", argv[1], cloud_src->points.size(), time.toc(), iterations);

    // 将原始点云src，执行包含旋转30°，平移z=1.0的刚性变化
    Eigen::Matrix4d t_matrix = Eigen::Matrix4d::Identity();

    // 设置旋转部分矩阵
    double theta = M_PI / 6;
    t_matrix(0, 0) = cos(theta);
    t_matrix(0, 1) = -sin(theta);
    t_matrix(1, 0) = sin(theta);
    t_matrix(1, 1) = cos(theta);

    // 设置平移部分，Z平移1m
    t_matrix(2, 3) += 1.0;

//    std::cout << "t_matrix： \n"<<t_matrix << std::endl;
    print4x4Matrix(t_matrix);

    // 对原始点云执行变换
    pcl::transformPointCloud(*cloud_src, *cloud_icp, t_matrix);

    *cloud_target = *cloud_icp;

    time.tic();
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    // 设置最大的迭代次数
    icp.setMaximumIterations(iterations);
    // 把平移旋转之后的点云作为输入点云
    icp.setInputSource(cloud_icp);
    // 把原来的原始点云作为目标点云，让cloud_icp不断的变换回来
    icp.setInputTarget(cloud_src);

    // 执行最多n次迭代的icp配置， cloud_icp是cloud_src经过变换后的点云
    icp.align(*cloud_icp);

    std::cout << "迭代了" << iterations << ", 时间：" << time.toc() << "ms" << std::endl;

    if (icp.hasConverged()) {
        PCL_INFO("ICP已经收敛了， 评分是%f\n", icp.getFitnessScore());
        t_matrix = icp.getFinalTransformation().cast<double>();
        print4x4Matrix(t_matrix);

    }else {
        PCL_ERROR("点云配准无法收敛");
        return -1;
    }

    // 用户按下一次空格就继续迭代一次

    pcl::visualization::PCLVisualizer viewer("3D Viewer");

    // 创建两个viewport
    int v1 (0);
    int v2 (1);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1); // 指定该视图在窗口的位置（左上和右下的比例）
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2); // 指定该视图在窗口的位置（左上和右下的比例）

    // 添加原始点云
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_src_color(cloud_src, 255, 255, 255);
    viewer.addPointCloud(cloud_src, cloud_src_color, "cloud_src_v1", v1);
    viewer.addPointCloud(cloud_src, cloud_src_color, "cloud_src_v2", v2);

    // 添加目标点云
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_target_color(cloud_target, 20, 180, 20);
    viewer.addPointCloud(cloud_target, cloud_target_color, "cloud_target_v1", v1);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color(cloud_icp, 180, 20, 20);
    viewer.addPointCloud(cloud_icp, cloud_icp_color, "cloud_icp_v2", v2);

    // 设置相机位置
    viewer.setCameraPosition(0.442587, -8.44861, 1.27435, -0.0413298, 0.0907372, 0.995017, 0);

    // 设置窗口尺寸
    viewer.setSize(1280, 1024);

    // 添加坐标系
    viewer.addCoordinateSystem(1.0);

    // 点云注册用户按键回调函数
    viewer.registerKeyboardCallback(&keyboardEventCallback, (void *)NULL);

    // 将每次配准的迭代次数设置为1
    icp.setMaximumIterations(1);

    while (!viewer.wasStopped()) {
        viewer.spinOnce();

        // 如果next_iteration 被设置为true，立刻修改回false。执行一次迭代
        if (next_iteration) {
            next_iteration = false;

            time.tic();
            icp.align(*cloud_icp);

            std::cout << "执行了一次迭代：" << time.toc() << "ms"<< std::endl;


            if (icp.hasConverged()) {

                iterations++;
                std::cout << "当前迭代的匹配分数：" << icp.getFitnessScore() << " 已迭代次数："<< iterations << std::endl;

                // 把每次的变换矩阵都进行累加（左乘）
                t_matrix *= icp.getFinalTransformation().cast<double>();
                print4x4Matrix(t_matrix);

                viewer.updatePointCloud(cloud_icp, cloud_icp_color, "cloud_icp_v2");

            }else {
                PCL_ERROR("结果没有收敛\n");
                return -1;
            }

        }
    }

    return 0;

}