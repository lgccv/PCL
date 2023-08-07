//
// Created by ty on 20-8-2.
//


#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

using namespace std;

/**
 * 点云的序列化
 */
int main(int argc, char *argv[]) {

    pcl::PointCloud<pcl::PointXYZ> cloud;

    cloud.width = 5;
    cloud.height = 1;
    cloud.is_dense = false;
    uint32_t points_count = cloud.width * cloud.height;
    cloud.points.resize(points_count);

    // 生成一些随机数， 写到文件
    for (int i = 0; i < points_count; ++i) {
        //  (0 -> 1.0) * 1024.0f >> 0 -> 1024.0f
        cloud.points[i].x = 1024.0f * (rand() / (RAND_MAX + 1.0f));
        cloud.points[i].y = 1024.0f * (rand() / (RAND_MAX + 1.0f));
        cloud.points[i].z = 1024.0f * (rand() / (RAND_MAX + 1.0f));
        std::cout <<
                "x: " << cloud.points[i].x <<
                " y: " << cloud.points[i].y <<
                " z: " << cloud.points[i].z <<
                std::endl;
    }
    pcl::io::savePCDFileASCII("./output/pcd_ascii.pcd", cloud);
    pcl::io::savePCDFileBinary("./output/pcd_binary.pcd", cloud);
    pcl::io::savePCDFileBinaryCompressed("./output/pcd_binary_compressed.pcd", cloud);
    pcl::io::savePCDFile("./output/pcd_save.pcd", cloud, true); // savePCDFileBinary


}