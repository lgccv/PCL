//
// Created by ty on 20-8-2.
/***
 * cloud viewer的使用例子
 * 回调函数
 * 设置背景色
 * 添加文字，形状
 */


#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

void viewerOnce(pcl::visualization::PCLVisualizer &visualizer){

    // 给点云设置个背景色
    visualizer.setBackgroundColor(1.0, 0.5, 1.0);

    pcl::PointXYZ center;
    center.x = 1.0;
    center.y = 1.0;
    center.z = 0;

    visualizer.addSphere(center, 0.25, "sphere", 0);
    std::cout << "viewer Once" << std::endl;

};

int counter = 0;
int user_counter = 0;
void viewerPsycho(pcl::visualization::PCLVisualizer &visualizer){

    // 先将之前添加的移除掉
    visualizer.removeShape("text_str", 0);

    stringstream ss;
    ss << "Per viewer loop: " << counter++;
    visualizer.addText(ss.str(), 100, 300, 22, 1.0,1.0,1.0, "text_str", 0);


    std::cout << "viewer Psycho: c->" << counter << " uc->" << user_counter << std::endl;
};

/**
 * 
 */
int main(int argc, char *argv[]) {

    // 加载点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::io::loadPCDFile("./data/pcl_logo.pcd", *cloud);

    // 可视化展示
    pcl::visualization::CloudViewer viewer("cloudViewer");

    viewer.showCloud(cloud);

    // 1. 在首次渲染时候执行操作（添加球体）
    viewer.runOnVisualizationThreadOnce(viewerOnce);

    // 2. 在每次更新渲染时候执行操作（添加文字）
    viewer.runOnVisualizationThread(viewerPsycho);

    while (!viewer.wasStopped()) {
        // 可以在此处更新点云
        user_counter++;
    }

    return 0;

}