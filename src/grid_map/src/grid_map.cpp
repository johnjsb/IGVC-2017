#include <ros/ros.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>

int main(int argc, char** argv){

    // Initialize node and publisher
    ros::init(argc, argv, "grid_map_demo");
    ros::NodeHandle nh("~");
    ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

    // Create grid map
    // Clion doesn't recognize {"elevation"} as a string vector of size 1?
    // instead sees it as a char array of size 10...
    std::vector<std::string> layers{"elevation"};
    grid_map::GridMap map(layers);
    map.setFrameId("map");
    map.setGeometry(grid_map::Length(1.2, 2.0), 0.03);
    ROS_INFO("Map of size %f x %f m (%i x %i cells")
    return 0;
}