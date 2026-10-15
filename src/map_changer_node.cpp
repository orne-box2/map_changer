#include <map_changer/map_changer_node.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>
#include <ros/master.h>
#include <ros/network.h>
#include <xmlrpcpp/XmlRpcClient.h>

namespace map_changer {
namespace {
std::string frame(std::string value)
{
    while (!value.empty() && value.front() == '/') value.erase(value.begin());
    return value;
}
class NodeClient : public XmlRpc::XmlRpcClient {
public:
    using XmlRpc::XmlRpcClient::XmlRpcClient;
    void poll() { _disp.work(0.01); }
};
bool node_rpc(const std::string& uri, const char* method, const XmlRpc::XmlRpcValue& args,
              XmlRpc::XmlRpcValue& result)
{
    std::string host;
    uint32_t port;
    if (!ros::network::splitURI(uri, host, port)) return false;
    NodeClient client(host.c_str(), port, "/");
    if (!client.executeNonBlock(method, args)) return false;
    const auto deadline = ros::WallTime::now() + ros::WallDuration(1.0);
    while (ros::ok() && ros::WallTime::now() < deadline) {
        client.poll();
        if (client.executeCheckDone(result))
            return !client.isFault() && result.getType() == XmlRpc::XmlRpcValue::TypeArray &&
                result.size() == 3 && result[0].getType() == XmlRpc::XmlRpcValue::TypeInt && int(result[0]) == 1;
    }
    client.close();
    return false;
}
}
map_changer_node::map_changer_node()
{
    std::string file_path, pose_topic, set_map_service, nomotion_service;
    pnh_.param("file_path", file_path, ros::package::getPath("map_changer") + "/config/tsukuba.yaml");
    pnh_.param("pose_timeout", pose_timeout_, 5.0);
    pnh_.param("check_map_bounds", check_map_bounds_, true);
    pnh_.param("map_margin", map_margin_, 0.6);
    pnh_.param("global_frame", global_frame_, std::string("map"));
    pnh_.param("robot_base_frame", robot_base_frame_, std::string("base_link"));
    pnh_.param("odom_frame", odom_frame_, std::string("odom"));
    pnh_.param("localization_node", localization_node_, std::string("/amcl"));
    pnh_.param("pose_topic", pose_topic, std::string("/amcl_pose"));
    pnh_.param("set_map_service", set_map_service, std::string("/set_map"));
    pnh_.param("nomotion_service", nomotion_service, std::string("/request_nomotion_update"));
    pnh_.param("replace_map_servers", replace_map_servers_, true);
    if (!pnh_.getParam("map_server_nodes", map_server_nodes_))
        map_server_nodes_ = {"/map_server", "/map_server_for_costmap"};
    pnh_.param("keep_tf_during_map_update", keep_tf_during_map_update_, true);
    pnh_.param("tf_bridge_timeout", tf_bridge_timeout_, 5.0);
    nh_.param(localization_node_ + "/transform_tolerance", transform_tolerance_, 0.5);
    global_frame_ = frame(global_frame_);
    robot_base_frame_ = frame(robot_base_frame_);
    odom_frame_ = frame(odom_frame_);
    if (global_frame_.empty()) throw std::runtime_error("global_frame must not be empty");
    if (robot_base_frame_.empty()) throw std::runtime_error("robot_base_frame must not be empty");
    if (odom_frame_.empty() || odom_frame_ == global_frame_)
        throw std::runtime_error("odom_frame must differ from global_frame");
    if (!std::isfinite(tf_bridge_timeout_) || tf_bridge_timeout_ <= 0 ||
        !std::isfinite(transform_tolerance_) || transform_tolerance_ < 0)
        throw std::runtime_error("Invalid TF bridge timeout/AMCL transform tolerance");
    transform_tolerance_ = std::max(0.1, transform_tolerance_);
    if (!std::isfinite(pose_timeout_) || pose_timeout_ <= 0)
        throw std::runtime_error("pose_timeout must be positive and finite");
    if (!std::isfinite(map_margin_) || map_margin_ < 0)
        throw std::runtime_error("map_margin must be nonnegative and finite");
    config_list_ = load_config(file_path);
    initial_map_ = load_initial_map(file_path);
    if (initial_map_ && check_map_bounds_) {
        initial_map_bounds_ = load_map_bounds(initial_map_->map_file);
        initial_costmap_bounds_ = initial_map_->costmap_file == initial_map_->map_file ?
            initial_map_bounds_ : load_map_bounds(initial_map_->costmap_file);
    }
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
    nomotion_srv_ = nh_.serviceClient<std_srvs::Empty>(nomotion_service);
    // Older navigation launches register both map servers under /change_map
    // and /static_map. Load and publish the new grids here so those launches
    // need no service remaps. Retire their processes only after a successful handover.
    const ros::SubscriberStatusCallback connected =
        [this](const ros::SingleSubscriberPublisher& subscriber) { map_connected(subscriber); };
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, connected, ros::SubscriberStatusCallback(), ros::VoidConstPtr(), true);
    costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map_for_costmap", 1, connected, ros::SubscriberStatusCallback(), ros::VoidConstPtr(), true);
    metadata_pub_ = nh_.advertise<nav_msgs::MapMetaData>("/map_metadata", 1, connected, ros::SubscriberStatusCallback(), ros::VoidConstPtr(), true);
    costmap_metadata_pub_ = nh_.advertise<nav_msgs::MapMetaData>("/map_metadata_for_costmap", 1, connected, ros::SubscriberStatusCallback(), ros::VoidConstPtr(), true);
    status_pub_ = nh_.advertise<std_msgs::String>("/map_changer/status", 1, true);
    status("idle");
    bridge_timer_ = nh_.createWallTimer(ros::WallDuration(0.01), &map_changer_node::bridge_tf, this);
    if (initial_map_)
        ROS_INFO("Initial map (independent of waypoint IDs): %s", initial_map_->map_file.c_str());
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
    // During handover (or with replacement disabled), old latched grids may
    // still arrive. Reassert the active data when RViz/costmaps reconnect.
    refresh_maps_ = true;
    refresh_after_ = ros::WallTime::now() + ros::WallDuration(0.2);
}
void map_changer_node::publish_maps()
{
    map_pub_.publish(active_map_);
    metadata_pub_.publish(active_map_->info);
    costmap_pub_.publish(active_costmap_);
    costmap_metadata_pub_.publish(active_costmap_->info);
}
bool map_changer_node::get_map(nav_msgs::GetMap::Request&, nav_msgs::GetMap::Response& response)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_active_maps_) return false;
    response.map = *active_map_;
    return true;
}
bool map_changer_node::get_costmap(nav_msgs::GetMap::Request&, nav_msgs::GetMap::Response& response)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_active_maps_) return false;
    response.map = *active_costmap_;
    return true;
}
void map_changer_node::bridge_tf(const ros::WallTimerEvent&)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!bridge_active_ || (ros::WallTime::now() - bridge_started_).toSec() > tf_bridge_timeout_) return;
    const auto now = ros::Time::now();
    if (now.isZero() || now + ros::Duration(0.1) < bridge_transform_.stamp_) return;
    // Keep the bridge horizon shorter than AMCL's normal future dating so its
    // next scan can take over immediately, including with a quantized sim clock.
    const auto stamp = now + ros::Duration(std::min(transform_tolerance_, 0.25));
    if (stamp <= bridge_last_stamp_) return;
    // Hold only the localization correction during synchronous AMCL map loading.
    // odom->base_link continues to track motion; never bridge a coordinate reset.
    bridge_transform_.stamp_ = stamp;
    tf_broadcaster_.sendTransform(bridge_transform_);
    bridge_last_stamp_ = stamp;
}
bool map_changer_node::set_localization_map(const MapConfig& config, const Transition& transition, const nav_msgs::OccupancyGrid& grid)
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
    if (config.reset_initialpose) {
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
    if (keep_tf_during_map_update_ && !config.reset_initialpose) {
        try {
            tf::StampedTransform transform;
            tf_listener_.lookupTransform(global_frame_, odom_frame_, ros::Time(0), transform);
            if (std::abs((ros::Time::now() - transform.stamp_).toSec()) <= pose_timeout_) {
                std::lock_guard<std::mutex> lock(mutex_);
                bridge_transform_ = transform;
                bridge_last_stamp_ = transform.stamp_;
                bridge_started_ = ros::WallTime::now();
                bridge_active_ = true;
            }
        } catch (const tf::TransformException& e) { ROS_WARN("TF bridge unavailable: %s", e.what()); }
    }
    const bool accepted = set_map_srv_.call(request) && request.response.success;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        bridge_active_ = false;
    }
    if (!accepted) {
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
bool map_changer_node::activate_map(const MapConfig& config, const Transition& transition)
{
    if (have_active_maps_ && active_map_file_ == config.map_file &&
        active_costmap_file_ == config.costmap_file && !config.reset_initialpose) return true;
    nav_msgs::OccupancyGrid::Ptr map(new nav_msgs::OccupancyGrid), costmap;
    try {
        *map = load_map(config.map_file, global_frame_);
        if (config.costmap_file == config.map_file) costmap = map;
        else {
            costmap.reset(new nav_msgs::OccupancyGrid(load_map(config.costmap_file, global_frame_)));
            // Companion YAMLs can reference the same image. Keep one raster in memory.
            if (costmap->info.resolution == map->info.resolution && costmap->info.width == map->info.width &&
                costmap->info.height == map->info.height && costmap->info.origin == map->info.origin &&
                costmap->data == map->data) costmap = map;
        }
    } catch (const std::exception& e) {
        fail(std::string("Could not load map: ") + e.what());
        return false;
    }
    if (!set_localization_map(config, transition, *map)) return false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        active_map_ = std::move(map);
        active_costmap_ = std::move(costmap);
        active_map_file_ = config.map_file;
        active_costmap_file_ = config.costmap_file;
        have_active_maps_ = true;
    }
    if (static_map_srv_.getService().empty())
        static_map_srv_ = nh_.advertiseService("/static_map", &map_changer_node::get_map, this);
    if (static_costmap_srv_.getService().empty())
        static_costmap_srv_ = nh_.advertiseService("/static_map_for_costmap", &map_changer_node::get_costmap, this);
    publish_maps();
    return true;
}
bool map_changer_node::initialize_map()
{
    if (!initial_map_ || initial_map_done_) return true;
    geometry_msgs::PoseWithCovarianceStamped pose;
    if (!set_map_srv_.exists() || !current_pose(pose)) {
        status("waiting for initial localization");
        ROS_INFO_THROTTLE(2.0, "Initial map pending: waiting for AMCL pose/service (set the actual pose in RViz)");
        if (ros::WallTime::now() >= next_nomotion_) {
            next_nomotion_ = ros::WallTime::now() + ros::WallDuration(1.0);
            if (nomotion_srv_.exists()) { std_srvs::Empty request; nomotion_srv_.call(request); }
        }
        return false;
    }
    const auto& p = pose.pose.pose.position;
    if (check_map_bounds_ && (!initial_map_bounds_.contains(p.x, p.y, map_margin_) ||
        !initial_costmap_bounds_.contains(p.x, p.y, map_margin_))) {
        status("waiting for initial map coverage");
        ROS_WARN_THROTTLE(2.0, "Initial map pending: pose %.3f, %.3f outside initial crop; check RViz pose/initial_map", p.x, p.y);
        return false;
    }
    const Transition initial{0, "startup", "initial_map", {}, ros::Time::now()};
    if (!activate_map(*initial_map_, initial)) {
        // A rejected service must not cause a continuous expensive reload loop.
        // Retry after restart once the reported cause has been corrected.
        initial_map_done_ = true;
        return true;
    }
    initial_map_done_ = true;
    status("initialized: " + initial_map_->map_file);
    ROS_INFO("Initial cropped map is active; waypoint 1 does not need to be visited");
    return true;
}
void map_changer_node::retire_map_servers()
{
    if (!replace_map_servers_ || !have_active_maps_ || ros::WallTime::now() < next_retirement_) return;
    next_retirement_ = ros::WallTime::now() + ros::WallDuration(2.0);
    ros::V_string nodes;
    if (!ros::master::getNodes(nodes)) return;
    for (const auto& configured : map_server_nodes_) {
        const std::string node = nh_.resolveName(configured);
        if (node == ros::this_node::getName() || std::find(nodes.begin(), nodes.end(), node) == nodes.end()) continue;
        XmlRpc::XmlRpcValue args, response, uri;
        args[0] = ros::this_node::getName(); args[1] = node;
        if (!ros::master::execute("lookupNode", args, response, uri, false) ||
            uri.getType() != XmlRpc::XmlRpcValue::TypeString) continue;
        XmlRpc::XmlRpcValue call, publications;
        call[0] = ros::this_node::getName();
        if (!node_rpc(std::string(uri), "getPublications", call, publications)) {
            ROS_WARN_THROTTLE(5.0, "Could not inspect original map server %s at %s", node.c_str(), std::string(uri).c_str());
            continue;
        }
        const auto& topics = publications[2];
        if (topics.getType() != XmlRpc::XmlRpcValue::TypeArray) continue;
        bool has_map = false, map_only = true;
        for (int i = 0; i < topics.size(); ++i) {
            if (topics[i].getType() != XmlRpc::XmlRpcValue::TypeArray || topics[i].size() != 2 ||
                topics[i][0].getType() != XmlRpc::XmlRpcValue::TypeString ||
                topics[i][1].getType() != XmlRpc::XmlRpcValue::TypeString) { map_only = false; break; }
            const std::string topic = topics[i][0], type = topics[i][1];
            const bool grid = (topic == "/map" || topic == "/map_for_costmap") && type == "nav_msgs/OccupancyGrid";
            const bool metadata = (topic == "/map_metadata" || topic == "/map_metadata_for_costmap") && type == "nav_msgs/MapMetaData";
            has_map = has_map || grid;
            if (!grid && !metadata && !(topic == "/rosout" && type == "rosgraph_msgs/Log")) map_only = false;
        }
        if (!has_map || !map_only) {
            ROS_WARN_THROTTLE(5.0, "Not retiring %s: publications are not limited to map-server topics", node.c_str());
            continue;
        }
        call[1] = "map_changer has taken over map publication and GetMap services";
        if (node_rpc(std::string(uri), "shutdown", call, response)) {
            ROS_INFO("Retired original map server %s; its previous map can be released", node.c_str());
            // Reassert active data after the final old latched deliveries.
            std::lock_guard<std::mutex> lock(mutex_);
            refresh_maps_ = true;
            refresh_after_ = ros::WallTime::now() + ros::WallDuration(0.2);
        } else {
            ROS_WARN_THROTTLE(5.0, "Could not retire %s; its previous map may still be retained", node.c_str());
        }
    }
}
void map_changer_node::loop()
{
    if (!initialize_map()) return;
    retire_map_servers();
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
    // Validate/load both grids before changing anything, then give AMCL the
    // exact localization grid instead of querying an ambiguous /static_map.
    if (!activate_map(config_list_[transition.config_index], transition)) return;
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
