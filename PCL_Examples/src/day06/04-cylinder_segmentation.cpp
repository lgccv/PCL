//
// Created by ty on 20-8-10.
//

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;
typedef pcl::PointXYZ PointT;

/**
 * 加载这个点云
 */
int main(int argc, char *argv[]) {

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr
            cloud_filtered(new pcl::PointCloud<PointT>),
            cloud_filtered2(new pcl::PointCloud<PointT>); // 只有圆柱体相关的点
    pcl::PointCloud<pcl::Normal>::Ptr
            cloud_normals(new pcl::PointCloud<pcl::Normal>),
            cloud_normals2(new pcl::PointCloud<pcl::Normal>); // 只有圆柱体相关的法线

    pcl::io::loadPCDFile("./data/table_scene_mug_stereo_textured.pcd", *cloud);

    pcl::console::print_highlight("点云点个数: %d\n", cloud->points.size());
//    过滤掉远于 1. 5 m 的数据点
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 1.5);
    pass.filter(*cloud_filtered);

    pcl::console::print_highlight("点云直通滤波后的点个数: %d\n", cloud_filtered->points.size());

//    估计每个点的表面法线
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(cloud_filtered);
    ne.setSearchMethod(kdtree);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    // 初始化内点索引列表
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
    // 创建平面分割模型系数
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients()), coefficients_cylinder(new pcl::ModelCoefficients());

//    分割出平面模型 （数据集中的桌面）并保存到磁盘中。
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.1);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normals);
    // 执行分割，将平面内点存储到inliers，模型系数存到cofficients
    seg.segment(*inliers_plane, *coefficients_plane);
    std::cout << "平面模型的系数：" << *coefficients_plane << std::endl;

    // 根据索引列表从点云中提取出平面点，保存到文件
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_plane);
    extract.setNegative(false);
    pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>);
    extract.filter(*cloud_plane);

    pcl::io::savePCDFileASCII("./data/table_scene_mug_stereo_textured_plane.pcd", *cloud_plane);


    extract.setNegative(true);
    extract.filter(*cloud_filtered2); // 只有杯子相关的点云了

    // 移除法向量中所有和平面相关的法线，只保留剩余的法线
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers_plane);
    extract_normals.setNegative(true);
    extract_normals.filter(*cloud_normals2); // 只有杯子相关的法线了

//    分割圆出柱体模型（数据集中的杯子）并保存到磁盘中。
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.05); // 内点到模型的距离
    seg.setRadiusLimits(0, 0.1);    // 设置圆柱体的半径范围
    seg.setInputCloud(cloud_filtered2);
    seg.setInputNormals(cloud_normals2);
    // 执行分割，将平面内点存储到inliers，模型系数存到cofficients
    seg.segment(*inliers_cylinder, *coefficients_cylinder);
    std::cout << "圆柱体模型的系数：" << *coefficients_cylinder << std::endl;

    // 将圆柱体提取出来
    extract.setInputCloud(cloud_filtered2);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);

    pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>);
    extract.filter(*cloud_cylinder);

    if (cloud_cylinder->points.empty()) {
        PCL_ERROR("没有找到圆柱体");
    }else {
        PCL_INFO("找到圆柱体，圆柱体点数：%d", cloud_cylinder->points.size());
        pcl::io::savePCDFileASCII("./data/table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder);
    }



}