#ifndef pointcloud_hpp
#define pointcloud_hpp

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>

class PointCloud {
private:
    std::vector<int> filenames;

    // void check_colors();

public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    
    bool load_from_file(int argc, char* argv[]);

};

#endif /* pointcloud_hpp */
