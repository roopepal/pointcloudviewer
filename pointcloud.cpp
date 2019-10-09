#include "pointcloud.hpp"
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <algorithm>
#include <initializer_list>
#include <iostream>

bool PointCloud::load_from_file(int argc, char* argv[]) {

    std::vector<int> filenames;
    bool file_is_pcd = false;
    
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");
    if (filenames.size () != 1)  {
        filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
        if (filenames.size () != 1) {
            return -1;
        } else {
            file_is_pcd = true;
        }
    }
    
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    if (file_is_pcd) {
        if (pcl::io::loadPCDFile (argv[filenames[0]], *cloud) < 0)  {
            std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
            return -1;
        }
    } else {
        if (pcl::io::loadPLYFile (argv[filenames[0]], *cloud) < 0)  {
            std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
            return -1;
        }
    }

    std::cout << "Loaded a cloud of " << cloud->size() << " points" << std::endl;
    
    remove_outliers();
    normalize();

    return true;

}

void PointCloud::get_dimensions(pcl::PointXYZ &min_pt, pcl::PointXYZ &max_pt, pcl::PointXYZ &center_pt) {

    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    center_pt = pcl::PointXYZ((min_pt.x + max_pt.x) / 2.0f, (min_pt.y + max_pt.y) / 2.0f , (min_pt.z + max_pt.z) / 2.0f);
    std::cout << "min_pt " << min_pt << " max_pt " << max_pt << " center_pt " << center_pt << std::endl;

}

void PointCloud::remove_outliers() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);

    cloud = cloud_filtered;

}

void PointCloud::normalize() {

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    //std::cout << "min_pt " << min_pt << " max_pt " << max_pt << std::endl;

    pcl::PointXYZ center_pt;
    center_pt.x = min_pt.x + (max_pt.x - min_pt.x) / 2.0;
    center_pt.y = min_pt.y + (max_pt.y - min_pt.y) / 2.0;
    center_pt.z = min_pt.z + (max_pt.z - min_pt.z) / 2.0;
    //std::cout << "center_pt " << center_pt << std::endl;

    for (auto &p : cloud->points) {
        p.x -= center_pt.x;
        p.y -= center_pt.y;
        p.z -= center_pt.z;
    }

    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    //std::cout << "min_pt " << min_pt << " max_pt " << max_pt << std::endl;

    float max_coord = std::max({max_pt.x, max_pt.y, max_pt.z});
    //std::cout << "max coord " << max_coord << std::endl;

    for (auto &p : cloud->points) {
        p.x /= max_coord;
        p.y /= max_coord;
        p.z /= max_coord;
    }
    
}