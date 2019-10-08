#include "display.hpp"
#include "pointcloud.hpp"
#include <iostream>

int main(int argc, char * argv[]) {
    
    PointCloud pc = PointCloud();
    pc.load_from_file(argc, argv);

    Display d = Display("PointCloud", 1152, 648);
    d.start(&pc, argc, argv);
    d.cleanup();

    return 0;
    
}
