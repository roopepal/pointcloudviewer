#ifndef pointcloud_hpp
#define pointcloud_hpp

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>

class PointCloud {
private:
    std::vector<int> filenames;

    void remove_outliers();
    void normalize();

public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    
    void get_dimensions(pcl::PointXYZ &min_pt, pcl::PointXYZ &max_pt, pcl::PointXYZ &center_pt);
    bool load_from_file(int argc, char* argv[]);

};

#endif /* pointcloud_hpp */
