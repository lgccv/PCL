//
// Created by ty on 20-8-4.
//


#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;


typedef pcl::PointXYZ PointType;

/**
 * 
 */
int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
        exit(0);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    cloud->width = 100;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        // -1.0 -> 1.0
        cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    if (strcmp(argv[1], "-r") == 0) {
        // 通过半径滤波执行, 如果一个点在指定半径内，有指定个数的邻居，则保留
        pcl::RadiusOutlierRemoval<PointType> outlierRemoval;
        outlierRemoval.setInputCloud(cloud);

        // 设置搜索半径
        outlierRemoval.setRadiusSearch(0.4);
        // 邻居个数
        outlierRemoval.setMinNeighborsInRadius(2);

        outlierRemoval.filter(*cloud_filtered);

    } else if ((strcmp(argv[1], "-c") == 0)) {
        // 条件滤波
        // 创建条件对象 0.0 < z <= 0.8 || x > 0
        pcl::ConditionAnd<PointType>::Ptr range_cond(new pcl::ConditionAnd<PointType>);

        // 添加比较对象 z > 0.0
        range_cond->addComparison(pcl::FieldComparison<PointType>::ConstPtr(new pcl::FieldComparison<PointType>(
                "z", pcl::ComparisonOps::GT, 0.0)));
        // 添加比较对象 z <= 0.8
        range_cond->addComparison(pcl::FieldComparison<PointType>::ConstPtr(new pcl::FieldComparison<PointType>(
                "z", pcl::ComparisonOps::LE, 0.8)));

        pcl::ConditionalRemoval<PointType> condRem;
        condRem.setInputCloud(cloud);
        // 设置过滤条件
        condRem.setCondition(range_cond);

        // false默认值：将条件之外的点移除掉
        // true：将条件之外的点还保留，但是设置NAN, 或者setUserFilterValue修改
        condRem.setKeepOrganized(false);
        condRem.filter(*cloud_filtered);


    } else {
        std::cerr << "请添加正确的参数： -r 半径滤波， -c 条件滤波" << std::endl;
        exit(-1);
    }


    std::cerr << "Cloud before filtering: " <<  cloud->points.size() << std::endl;
    for (size_t i = 0; i < cloud->points.size(); ++i)
        std::cerr << "    " << cloud->points[i].x << " "
                  << cloud->points[i].y << " "
                  << cloud->points[i].z << std::endl;

    // display pointcloud after filtering
    std::cerr << "Cloud after filtering: " << cloud_filtered->points.size() << std::endl;
    for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
        std::cerr << "    " << cloud_filtered->points[i].x << " "
                  << cloud_filtered->points[i].y << " "
                  << cloud_filtered->points[i].z << std::endl;


    pcl::visualization::PCLVisualizer viewer("3d_viewer");

    pcl::visualization::PointCloudColorHandlerCustom<PointType> cloud_color(cloud, 0, 255, 0);
    viewer.addPointCloud(cloud, cloud_color, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

    pcl::visualization::PointCloudColorHandlerCustom<PointType> cloud_filtered_color(cloud, 255, 0, 0);
    viewer.addPointCloud(cloud_filtered, cloud_filtered_color, "cloud_filtered");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud_filtered");

    viewer.addCoordinateSystem(1.0);

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

}