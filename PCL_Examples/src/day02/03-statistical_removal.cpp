//
// Created by ty on 20-8-4.
//


#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace std;

/**
 * 
 */
int main(int argc, char *argv[]) {


    PointCloud::Ptr cloud(new PointCloud);
    PointCloud::Ptr cloud_filtered(new PointCloud);

    pcl::PCDReader reader;
    reader.read<PointT>("./data/table_scene_lms400_downsample.pcd", *cloud);

    // 创建统计学离群点移除过滤器
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    // 设置过滤邻域K
    sor.setMeanK(50);
    // 设置标准差的系数, 值越大，丢掉的点越少
    sor.setStddevMulThresh(1.0f);
    sor.filter(*cloud_filtered);

    // 保存去掉离群点之后的点云
    pcl::PCDWriter writer;
    writer.write("./data/table_scene_lms400_inliers.pcd", *cloud_filtered);

    // 取反
    sor.setNegative(true);
    sor.filter(*cloud_filtered);
    // 保存被去掉离群点
    writer.write("./data/table_scene_lms400_outliers.pcd", *cloud_filtered);


}