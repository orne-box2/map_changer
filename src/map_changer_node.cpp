#include <map_changer/map_changer_node.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>

namespace map_changer {
namespace {
std::string frame(std::string value)
{
    while (!value.empty() && value.front() == '/') value.erase(value.begin());
    return value;
}
}
map_changer_node::map_changer_node()
{
    std::string file_path, pose_topic, set_map_service;
    pnh_.param("file_path", file_path, ros::package::getPath("map_changer") + "/config/test1.yaml");
    pnh_.param("pose_timeout", pose_timeout_, 5.0);
    pnh_.param("check_map_bounds", check_map_bounds_, true);
    pnh_.param("map_margin", map_margin_, 0.6);
    pnh_.param("global_frame", global_frame_, std::string("map"));
    pnh_.param("robot_base_frame", robot_base_frame_, std::string("base_link"));
    pnh_.param("localization_node", localization_node_, std::string("/amcl"));
    pnh_.param("pose_topic", pose_topic, std::string("/amcl_pose"));
    pnh_.param("set_map_service", set_map_service, std::string("/set_map"));
    global_frame_ = frame(global_frame_);
    robot_base_frame_ = frame(robot_base_frame_);
    if (global_frame_.empty()) throw std::runtime_error("global_frame must not be empty");
    if (robot_base_frame_.empty()) throw std::runtime_error("robot_base_frame must not be empty");
    if (!std::isfinite(pose_timeout_) || pose_timeout_ <= 0)
        throw std::runtime_error("pose_timeout must be positive and finite");
    if (!std::isfinite(map_margin_) || map_margin_ < 0)
        throw std::runtime_error("map_margin must be nonnegative and finite");
    config_list_ = load_config(file_path);
    if (check_map_bounds_) {
        for (const auto& config : config_list_) {
            map_bounds_.push_back(load_map_bounds(config.map_file));
            costmap_bounds_.push_back(config.costmap_file == config.map_file ? map_bounds_.back() : load_map_bounds(config.costmap_file));
        }
    }
    for (const auto& config : config_list_)
        ROS_INFO("After waypoint %s: map=%s, costmap=%s, pose=%s", config.waypoint_id.c_str(),
            config.map_file.c_str(), config.costmap_file.c_str(),
            config.reset_initialpose ? "next waypoint coordinates" : "current localized pose");
    wp_sub_ = nh_.subscribe("/waypoint_manager/waypoint", 100, &map_changer_node::cb_wp, this);
    pose_sub_ = nh_.subscribe(pose_topic, 1, &map_changer_node::cb_pose, this);
    route_sub_ = nh_.subscribe("/waypoint_manager/route", 1, &map_changer_node::cb_route, this);
    set_map_srv_ = nh_.serviceClient<nav_msgs::SetMap>(set_map_service);
    // Older navigation launches register both map servers under /change_map
    // and /static_map. Load and publish the new grids here so those launches
    // need no service remaps or map-server restarts.
    const ros::SubscriberStatusCallback connected =
        [this](const ros::SingleSubscriberPublisher& subscriber) { map_connected(subscriber); };
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, connected, ros::SubscriberStatusCallback(), ros::VoidConstPtr(), true);
    costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map_for_costmap", 1, connected, ros::SubscriberStatusCallback(), ros::VoidConstPtr(), true);
    metadata_pub_ = nh_.advertise<nav_msgs::MapMetaData>("/map_metadata", 1, connected, ros::SubscriberStatusCallback(), ros::VoidConstPtr(), true);
    costmap_metadata_pub_ = nh_.advertise<nav_msgs::MapMetaData>("/map_metadata_for_costmap", 1, connected, ros::SubscriberStatusCallback(), ros::VoidConstPtr(), true);
    status_pub_ = nh_.advertise<std_msgs::String>("/map_changer/status", 1, true);
    status("idle");
    ROS_INFO("Loaded %zu map transitions from %s (no navigation pause/cancel)", config_list_.size(), file_path.c_str());
}
void map_changer_node::cb_wp(const waypoint_manager_msgs::Waypoint::ConstPtr& msg)
{
    // A non-stop waypoint can be advanced by radius_node before move_base ever
    // reports SUCCEEDED. Use the observed waypoint transition instead.
    std::lock_guard<std::mutex> lock(mutex_);
    if (wp_ && wp_->identity != msg->identity) {
        const auto entry = std::find_if(config_list_.begin(), config_list_.end(),
            [this](const MapConfig& config) { return config.waypoint_id == wp_->identity; });
        if (entry != config_list_.end()) {
            transitions_.push_back({static_cast<std::size_t>(std::distance(config_list_.begin(), entry)),
                wp_->identity, msg->identity, msg->pose, ros::Time::now()});
            ROS_INFO("Waypoint advanced %s -> %s; queued map change", wp_->identity.c_str(), msg->identity.c_str());
        }
    }
    // Repeated publications of the same waypoint do not queue another change.
    wp_ = msg;
}
void map_changer_node::cb_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    if (frame(msg->header.frame_id) != global_frame_) return;
    const auto& pose = msg->pose.pose;
    const auto& q = pose.orientation;
    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) || !std::isfinite(q.x) ||
        !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
        std::abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) > 1e-3) return;
    if (!std::all_of(msg->pose.covariance.begin(), msg->pose.covariance.end(),
        [](double value) { return std::isfinite(value); }) || msg->pose.covariance[0] < 0 ||
        msg->pose.covariance[7] < 0 || msg->pose.covariance[35] < 0) return;
    std::lock_guard<std::mutex> lock(mutex_);
    current_pose_ = msg;
    pose_received_ = ros::WallTime::now();
}
void map_changer_node::cb_route(const waypoint_manager_msgs::Route::ConstPtr& msg)
{
    for (const auto& config : config_list_)
        if (std::find(msg->identities.begin(), msg->identities.end(), config.waypoint_id) == msg->identities.end())
            ROS_WARN("Configured switch waypoint %s is absent from the current route; check file_path and route_file",
                config.waypoint_id.c_str());
}
bool map_changer_node::current_pose(geometry_msgs::PoseWithCovarianceStamped& pose)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!current_pose_) return false;
    const auto stamp = current_pose_->header.stamp;
    if ((ros::WallTime::now() - pose_received_).toSec() <= pose_timeout_ &&
        (stamp.isZero() || (ros::Time::now() - stamp).toSec() <= pose_timeout_)) {
        pose = *current_pose_;
        return true;
    }
    // AMCL does not publish a new pose while the robot is stationary. After an
    // intentional stop, use its maintained map->odom transform and fresh odometry
    // instead of rejecting the unchanged estimate or reusing an expired stamp.
    tf::StampedTransform transform;
    try {
        tf_listener_.lookupTransform(global_frame_, robot_base_frame_, ros::Time(0), transform);
    } catch (const tf::TransformException&) { return false; }
    const double age = (ros::Time::now() - transform.stamp_).toSec();
    if (transform.stamp_.isZero() || age < -pose_timeout_ || age > pose_timeout_) return false;
    const auto& position = transform.getOrigin();
    const auto& rotation = transform.getRotation();
    if (!std::isfinite(position.x()) || !std::isfinite(position.y()) || !std::isfinite(position.z()) ||
        !std::isfinite(rotation.x()) || !std::isfinite(rotation.y()) || !std::isfinite(rotation.z()) ||
        !std::isfinite(rotation.w()) || std::abs(rotation.length2() - 1.0) > 1e-3) return false;
    pose = *current_pose_;
    pose.header.stamp = transform.stamp_;
    tf::pointTFToMsg(transform.getOrigin(), pose.pose.pose.position);
    tf::quaternionTFToMsg(transform.getRotation(), pose.pose.pose.orientation);
    return true;
}
void map_changer_node::status(const std::string& value)
{
    std_msgs::String msg;
    msg.data = value;
    status_pub_.publish(msg);
}
void map_changer_node::fail(const std::string& message)
{
    status("error: " + message);
    ROS_ERROR("Map transition failed: %s", message.c_str());
    // Map changing never controls waypoint_server, cancels a goal, or writes
    // cmd_vel, including on failure. Navigation remains under its own control.
}
void map_changer_node::map_connected(const ros::SingleSubscriberPublisher&)
{
    std::lock_guard<std::mutex> lock(mutex_);
    // Other map servers retain their original latched grids. Send the active
    // grids again after their initial deliveries when RViz/costmaps reconnect.
    refresh_maps_ = true;
    refresh_after_ = ros::WallTime::now() + ros::WallDuration(0.2);
}
void map_changer_node::publish_maps()
{
    map_pub_.publish(active_map_);
    metadata_pub_.publish(active_map_.info);
    costmap_pub_.publish(active_costmap_);
    costmap_metadata_pub_.publish(active_costmap_.info);
}
bool map_changer_node::get_map(nav_msgs::GetMap::Request&, nav_msgs::GetMap::Response& response)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_active_maps_) return false;
    response.map = active_map_;
    return true;
}
bool map_changer_node::get_costmap(nav_msgs::GetMap::Request&, nav_msgs::GetMap::Response& response)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_active_maps_) return false;
    response.map = active_costmap_;
    return true;
}
bool map_changer_node::set_localization_map(const Transition& transition, const nav_msgs::OccupancyGrid& grid)
{
    if (!grid.info.width || !grid.info.height || !std::isfinite(grid.info.resolution) || grid.info.resolution <= 0 ||
        grid.data.size() != static_cast<std::size_t>(grid.info.width) * grid.info.height ||
        frame(grid.header.frame_id) != global_frame_) {
        fail("Invalid localization map; check the map YAML/image/frame");
        return false;
    }
    nav_msgs::SetMap request;
    request.request.map = grid;
    auto& pose = request.request.initial_pose;
    if (config_list_[transition.config_index].reset_initialpose) {
        pose.pose.pose = transition.target_pose;
        pose.pose.pose.position.z = 0;
        pose.pose.covariance[0] = pose.pose.covariance[7] = 0.25;
        pose.pose.covariance[35] = std::pow(std::acos(-1.0) / 12.0, 2);
        pose.header.stamp = transition.reached_time;
    } else if (!current_pose(pose)) {
        fail("No fresh localized pose: check pose_topic (default /amcl_pose)");
        return false;
    }
    pose.header.frame_id = global_frame_;
    // Preserve the measurement/transition time while moving. AMCL uses the
    // intervening odometry to project this pose to the service execution time.
    // Unstamped inputs have no measurement time, so use the current ROS time.
    if (pose.header.stamp.isZero()) pose.header.stamp = ros::Time::now();
    if (!set_map_srv_.call(request) || !request.response.success) {
        fail("Localization rejected set_map: " + set_map_srv_.getService());
        return false;
    }
    const auto& position = pose.pose.pose;
    nh_.setParam(localization_node_ + "/initial_pose_x", position.position.x);
    nh_.setParam(localization_node_ + "/initial_pose_y", position.position.y);
    nh_.setParam(localization_node_ + "/initial_pose_a", tf::getYaw(position.orientation));
    ROS_INFO("Localization map/pose updated: x=%.3f, y=%.3f, yaw=%.3f",
        position.position.x, position.position.y, tf::getYaw(position.orientation));
    return true;
}
void map_changer_node::loop()
{
    bool refresh = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (refresh_maps_ && ros::WallTime::now() >= refresh_after_) {
            refresh_maps_ = false;
            refresh = have_active_maps_;
        }
    }
    if (refresh) publish_maps();
    Transition transition;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (transitions_.empty()) return;
        transition = transitions_.front();
        transitions_.pop_front();
    }
    if (!set_map_srv_.exists()) {
        fail("Localization service missing: " + set_map_srv_.getService());
        return;
    }
    geometry_msgs::PoseWithCovarianceStamped pose;
    if (!config_list_[transition.config_index].reset_initialpose && !current_pose(pose)) {
        fail("No fresh localized pose: check pose_topic (default /amcl_pose)");
        return;
    }
    if (check_map_bounds_ && !config_list_[transition.config_index].reset_initialpose) {
        const auto& position = pose.pose.pose.position;
        if (!map_bounds_[transition.config_index].contains(position.x, position.y, map_margin_) ||
            !costmap_bounds_[transition.config_index].contains(position.x, position.y, map_margin_)) {
            // radius_node may advance just outside the next cropped map. Keep
            // navigating on the old map until the robot enters the overlap.
            std::lock_guard<std::mutex> lock(mutex_);
            transitions_.push_front(transition);
            status("waiting for map coverage: " + transition.source_id + " -> " + transition.target_id);
            ROS_INFO_THROTTLE(2.0, "Map change pending: current pose %.3f, %.3f is outside the new map margin", position.x, position.y);
            return;
        }
    }
    status("changing: " + transition.source_id + " -> " + transition.target_id);
    nav_msgs::OccupancyGrid map, costmap;
    try {
        const auto& config = config_list_[transition.config_index];
        map = load_map(config.map_file, global_frame_);
        costmap = config.costmap_file == config.map_file ? map : load_map(config.costmap_file, global_frame_);
    } catch (const std::exception& e) {
        fail(std::string("Could not load map: ") + e.what());
        return;
    }
    // Validate/load both grids before changing anything, then give AMCL the
    // exact localization grid instead of querying an ambiguous /static_map.
    if (!set_localization_map(transition, map)) return;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        active_map_ = std::move(map);
        active_costmap_ = std::move(costmap);
        have_active_maps_ = true;
    }
    // Until the first successful switch, the existing servers remain the only
    // source of maps. Afterwards new GetMap clients must receive the active grid.
    if (static_map_srv_.getService().empty())
        static_map_srv_ = nh_.advertiseService("/static_map", &map_changer_node::get_map, this);
    if (static_costmap_srv_.getService().empty())
        static_costmap_srv_ = nh_.advertiseService("/static_map_for_costmap", &map_changer_node::get_costmap, this);
    publish_maps();
    status("changed: " + transition.source_id + " -> " + transition.target_id);
    ROS_INFO("Map transition completed after waypoint %s", transition.source_id.c_str());
}
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_changer_node");
    try {
        map_changer::map_changer_node node;
        // Map service calls can take time; keep incoming poses and waypoint
        // transitions responsive on a separate callback thread.
        ros::AsyncSpinner spinner(1);
        spinner.start();
        ros::WallRate rate(20);
        while (ros::ok()) { node.loop(); rate.sleep(); }
        spinner.stop();
    } catch (const std::exception& e) { ROS_FATAL("map_changer: %s", e.what()); return 1; }
    return 0;
}
