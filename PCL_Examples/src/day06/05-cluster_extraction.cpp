//
// Created by ty on 20-8-10.
//
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/print.h>

using namespace std;
typedef  pcl::PointXYZ PointT;

/**
 * 聚类提取
 */
int main(int argc, char *argv[]) {

    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    reader.read("./data/tabletop.pcd", *cloud);
    PCL_INFO("原始点云的点个数：%d \n", cloud->points.size());

    // 执行降采样
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.01f, 0.01f, 0.01f);
    voxel.filter(*cloud_filtered);

    PCL_INFO("降采样之后的点个数：%d \n", cloud_filtered->points.size());

    // 创建SAC模型分割器
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
    // 设置最大迭代次数
    seg.setMaxIterations(100);
    // 设置距离阈值
    seg.setDistanceThreshold(0.02);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

    int org_points_count = cloud_filtered->points.size();
    // 循环去除平面
    // 1. 直到剩余点的个数 <= 所有点的30%
    // 2. 找不到平面的时候
    while (cloud_filtered->points.size() > 0.3 * org_points_count) {

        // 设置输入点云
        seg.setInputCloud(cloud_filtered);
        // 执行分割，把分割出来的点云索引保存到inliers里，把模型系数存到cofficients
        seg.segment(*inliers, *cofficients);

        if (inliers->indices.size() < 10) {
            std::cout << "不能从剩余的点云中找到任何平面" << std::endl;
            break;
        }

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);

        extract.setNegative(false);
        extract.filter(*cloud_plane);
        PCL_INFO("有一个平面所有点被去除：%d \n", cloud_plane->points.size());

        extract.setNegative(true); // 去掉平面，直流剩余的部分
        extract.filter(*cloud_f);

        *cloud_filtered = *cloud_f;
    }

    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
    kdtree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    // 创建欧式聚类提取对象
    pcl::EuclideanClusterExtraction<PointT> ece;
    ece.setClusterTolerance(0.02); // 设置邻近搜索的半径 2cm
    ece.setMinClusterSize(100); // 设置簇（集群）最小大小
    ece.setMaxClusterSize(25000); // 设置簇（集群）最大大小
    ece.setSearchMethod(kdtree);
    ece.setInputCloud(cloud_filtered);
    ece.extract(cluster_indices); // 将每一组簇（子点云）保存到参数列表中

    pcl::visualization::PCLVisualizer viewer("3DViewer");

    for (int i = 0; i < cluster_indices.size(); ++i) {
        pcl::PointIndices &indices = cluster_indices[i];
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_single(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud_filtered, indices, *cloud_single);

        pcl::console::print_highlight("加载第 %d 个点云，共有点个数： %d\n",i, cloud_single->points.size());

        stringstream ss;
        ss << "cluster_single_" << i ;
        pcl::visualization::PointCloudColorHandlerRandom<PointT> color_rad(cloud_single);
        viewer.addPointCloud(cloud_single, color_rad, ss.str());
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss.str());
    }

    viewer.addCoordinateSystem(0.5);
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

}