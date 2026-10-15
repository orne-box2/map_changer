#ifndef MAP_CHANGER_CONFIG_H
#define MAP_CHANGER_CONFIG_H
#include <string>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>
namespace map_changer {
struct MapConfig {
    std::string waypoint_id, map_file, costmap_file;
    // Cropped maps share coordinates; resetting to the next waypoint is opt-in.
    bool reset_initialpose = false;
};
struct MapBounds {
    double x, y, yaw, width, height;
    bool contains(double pose_x, double pose_y, double margin) const;
};
std::vector<MapConfig> load_config(const std::string& file_path);
MapBounds load_map_bounds(const std::string& map_file);
nav_msgs::OccupancyGrid load_map(const std::string& map_file, const std::string& frame);
}
#endif
