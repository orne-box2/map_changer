#ifndef MAP_CHANGER_NODE_H
#define MAP_CHANGER_NODE_H
#include <ros/ros.h>
#include <map_changer/config.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/SetMap.h>
#include <waypoint_manager_msgs/Waypoint.h>
#include <waypoint_manager_msgs/Route.h>
#include <deque>
#include <mutex>
#include <tf/transform_listener.h>

namespace map_changer {
class map_changer_node {
public:
    map_changer_node();
    void loop();
private:
    struct Transition {
        std::size_t config_index;
        std::string source_id, target_id;
        geometry_msgs::Pose target_pose;
        ros::Time reached_time;
    };
    void cb_wp(const waypoint_manager_msgs::Waypoint::ConstPtr& msg);
    void cb_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void cb_route(const waypoint_manager_msgs::Route::ConstPtr& msg);
    bool current_pose(geometry_msgs::PoseWithCovarianceStamped& pose);
    bool set_localization_map(const Transition& transition, const nav_msgs::OccupancyGrid& map);
    void map_connected(const ros::SingleSubscriberPublisher& subscriber);
    void publish_maps();
    bool get_map(nav_msgs::GetMap::Request& request, nav_msgs::GetMap::Response& response);
    bool get_costmap(nav_msgs::GetMap::Request& request, nav_msgs::GetMap::Response& response);
    void status(const std::string& value);
    void fail(const std::string& message);
    ros::NodeHandle nh_, pnh_{"~"};
    ros::Subscriber wp_sub_, pose_sub_, route_sub_;
    ros::ServiceClient set_map_srv_;
    ros::ServiceServer static_map_srv_, static_costmap_srv_;
    ros::Publisher status_pub_, map_pub_, costmap_pub_, metadata_pub_, costmap_metadata_pub_;
    nav_msgs::OccupancyGrid active_map_, active_costmap_;
    bool have_active_maps_ = false, refresh_maps_ = false;
    ros::WallTime refresh_after_;
    std::vector<MapConfig> config_list_;
    std::vector<MapBounds> map_bounds_, costmap_bounds_;
    waypoint_manager_msgs::Waypoint::ConstPtr wp_;
    geometry_msgs::PoseWithCovarianceStamped::ConstPtr current_pose_;
    ros::WallTime pose_received_;
    std::deque<Transition> transitions_;
    std::mutex mutex_;
    std::string global_frame_, robot_base_frame_, localization_node_;
    double pose_timeout_;
    double map_margin_;
    bool check_map_bounds_;
    tf::TransformListener tf_listener_;
};
}
#endif
