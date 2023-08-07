//
// Created by ty on 20-8-4.
//


#include <iostream>
#include <pcl/io/io.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

using namespace std;

/**
 * 通过点云生成深度图
 */
int main(int argc, char *argv[]) {


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> &pointCloud = *cloud;

    // 添加一个正方形区域 -0.5 -> 0.5
    for (float y = -0.5f; y <= 0.5f; y += 0.01f) {
        for (float z = -0.5f; z <= 0.5f; z += 0.01f) {
            pcl::PointXYZ point;
            point.x = 2.0f - y;
            point.y = y;
            point.z = z;
            pointCloud.points.push_back(point);
        }
    }
    uint32_t size = pointCloud.points.size();
    pointCloud.width = size;
    pointCloud.height = 1;

//    pcl::io::loadPCDFile("./data/table_scene_lms400_downsample.pcd", pointCloud);


    float DE2RA = M_PI / 180.0f;
    float RA2DE = 180.0f / M_PI;
//    * \param angular_resolution the angular difference (in radians) between the individual pixels in the image
    // angular_resolution： 扫描的分辨率，1° -> 弧度。
    float angular_resolution =  DEG2RAD(1.0f); // degrees to radians

//    * \param max_angle_width an angle (in radians) defining the horizontal bounds of the sensor
    // max_angle_width 水平方向扫描的范围 360° -> 弧度
    float max_angle_width = 360.0f * DE2RA;

//    * \param max_angle_height an angle (in radians) defining the vertical bounds of the sensor
    // max_angle_height 竖直方向扫描范围 180° -> 弧度
    float max_angle_height = 180.0f * DE2RA;

//    * \param sensor_pose an affine matrix defining the pose of the sensor (defaults to
//    *                    Eigen::Affine3f::Identity () )
    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, -3.0f, 0.0f);

//    * \param coordinate_frame the coordinate frame (defaults to CAMERA_FRAME)
    pcl::RangeImage::CoordinateFrame coordinateFrame = pcl::RangeImage::CAMERA_FRAME;

//    * \param noise_level - The distance in meters inside of which the z-buffer will not use the minimum,
//    *                      but the mean of the points. If 0.0 it is equivalent to a normal z-buffer and will always take the minimum per cell.
    float noise_level = 0.0f;

    float minRange = 0.0f;
//    * \param border_size the border size (defaults to 0)
    float border_size = 0.0f;
    // 从点云图生成深度图
    pcl::RangeImage::Ptr rangeImagePtr(new pcl::RangeImage);
    pcl::RangeImage &rangeImage = *rangeImagePtr;

    rangeImage.createFromPointCloud(pointCloud, angular_resolution, max_angle_width, max_angle_height, sensorPose,
        coordinateFrame, noise_level, minRange, border_size
    );

    pcl::visualization::PCLVisualizer viewer("3dViewer");
    viewer.setBackgroundColor(1.0, 1.0, 1.0);

    // 添加的原始点云
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 100, 0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample_cloud");

    // 添加深度图
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_color(rangeImagePtr, 0, 0, 0);
    viewer.addPointCloud(rangeImagePtr, range_color, "range_image");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "range_image");

    viewer.addCoordinateSystem(1.0);

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
        pcl_sleep(0.01);
    }


}