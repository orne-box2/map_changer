#include <map_changer/config.h>
#include <filesystem>
#include <cmath>
#include <set>
#include <stdexcept>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <map_server/image_loader.h>

namespace map_changer {
namespace {
namespace fs = std::filesystem;
std::string scalar(const YAML::Node& node, const std::string& key, const std::string& context)
{
    const YAML::Node value = node[key];
    if (!value || !value.IsScalar() || value.as<std::string>().empty()) {
        std::string message = context + "." + key + " must be a non-empty scalar";
        if (key == "waypoint_id" && node["wayponit_id"]) message += " (rename wayponit_id to waypoint_id)";
        throw std::runtime_error(message);
    }
    return value.as<std::string>();
}
std::string map_path(std::string value, const fs::path& directory, const std::string& context)
{
    const std::string prefix = "package://";
    if (value.compare(0, prefix.size(), prefix) == 0) {
        const auto slash = value.find('/', prefix.size());
        if (slash == std::string::npos) throw std::runtime_error(context + ": expected package://PACKAGE/path");
        const auto package = ros::package::getPath(value.substr(prefix.size(), slash - prefix.size()));
        if (package.empty()) throw std::runtime_error(context + ": ROS package not found in " + value);
        value = (fs::path(package) / value.substr(slash + 1)).string();
    }
    fs::path path(value);
    if (path.extension() != ".yaml" && path.extension() != ".yml") path += ".yaml";
    if (path.is_relative()) path = directory / path;
    path = fs::absolute(path).lexically_normal();
    if (!fs::is_regular_file(path)) throw std::runtime_error(context + ": map YAML does not exist: " + path.string());
    return path.string();
}
}
std::vector<MapConfig> load_config(const std::string& file_path)
{
    const auto root = YAML::LoadFile(file_path);
    if (!root.IsMap() || !root["config"] || !root["config"].IsSequence() || root["config"].size() == 0)
        throw std::runtime_error(file_path + ": config must be a non-empty sequence of waypoint_id/map_file entries");
    std::vector<MapConfig> result;
    std::set<std::string> ids;
    const auto directory = fs::absolute(file_path).parent_path();
    const YAML::Node entries = root["config"];
    for (std::size_t i = 0; i < entries.size(); ++i) {
        const YAML::Node entry = entries[i];
        const auto context = file_path + ": config[" + std::to_string(i) + "]";
        if (!entry.IsMap()) throw std::runtime_error(context + " must be a mapping");
        MapConfig config;
        config.waypoint_id = scalar(entry, "waypoint_id", context);
        if (!ids.insert(config.waypoint_id).second) throw std::runtime_error(context + ": duplicate waypoint_id " + config.waypoint_id);
        config.map_file = map_path(scalar(entry, "map_file", context), directory, context + ".map_file");
        if (entry["costmap_file"]) {
            config.costmap_file = map_path(scalar(entry, "costmap_file", context), directory, context + ".costmap_file");
        } else {
            const fs::path path(config.map_file);
            const auto sibling = path.parent_path() / (path.stem().string() + "_for_costmap" + path.extension().string());
            config.costmap_file = fs::is_regular_file(sibling) ? sibling.string() : config.map_file;
        }
        if (entry["reset_initialpose"]) config.reset_initialpose = entry["reset_initialpose"].as<bool>();
        result.push_back(config);
    }
    return result;
}
nav_msgs::OccupancyGrid load_map(const std::string& map_file, const std::string& frame)
{
    const auto metadata = YAML::LoadFile(map_file);
    const auto origin = metadata["origin"];
    if (!origin.IsSequence() || origin.size() != 3)
        throw std::runtime_error(map_file + ": origin must contain x, y, yaw");
    double values[] = {origin[0].as<double>(), origin[1].as<double>(), origin[2].as<double>()};
    const double resolution = metadata["resolution"].as<double>();
    if (!std::isfinite(resolution) || resolution <= 0 || !std::isfinite(values[0]) ||
        !std::isfinite(values[1]) || !std::isfinite(values[2]))
        throw std::runtime_error(map_file + ": invalid map geometry");
    std::filesystem::path image(metadata["image"].as<std::string>());
    if (image.is_relative()) image = std::filesystem::path(map_file).parent_path() / image;
    nav_msgs::GetMap::Response grid;
    map_server::loadMapFromFile(&grid, image.c_str(), resolution, metadata["negate"].as<int>() != 0,
        metadata["occupied_thresh"].as<double>(), metadata["free_thresh"].as<double>(), values);
    if (!grid.map.info.width || !grid.map.info.height)
        throw std::runtime_error(map_file + ": map image must not be empty");
    grid.map.header.frame_id = frame;
    grid.map.header.stamp = ros::Time::now();
    grid.map.info.map_load_time = grid.map.header.stamp;
    return grid.map;
}
MapBounds load_map_bounds(const std::string& map_file)
{
    const auto grid = load_map(map_file, "map");
    const auto origin = YAML::LoadFile(map_file)["origin"];
    return {origin[0].as<double>(), origin[1].as<double>(), origin[2].as<double>(),
        grid.info.width * grid.info.resolution, grid.info.height * grid.info.resolution};
}
bool MapBounds::contains(double pose_x, double pose_y, double margin) const
{
    const double dx = pose_x - x, dy = pose_y - y;
    const double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    return local_x >= margin && local_y >= margin && local_x < width - margin && local_y < height - margin;
}
}
