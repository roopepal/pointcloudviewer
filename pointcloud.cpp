#include "pointcloud.hpp"
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>

// void PointCloud::check_colors() {

//     bool found_rgb = false;
//     for (auto field : cloud->)
    

// }

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
    
    return true;

}
