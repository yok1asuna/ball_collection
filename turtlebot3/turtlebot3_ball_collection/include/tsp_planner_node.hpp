#ifndef TURTLEBOT3_BALL_COLLECTION__TSP_PLANNER_NODE_HPP_
#define TURTLEBOT3_BALL_COLLECTION__TSP_PLANNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>

class TspPlannerNode : public rclcpp::Node
{
public:
  TspPlannerNode();

private:
  void centroids_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  std::vector<geometry_msgs::msg::Point> solve_tsp(const std::vector<geometry_msgs::msg::Point>& points);
  void publish_path(const std::vector<geometry_msgs::msg::Point>& path);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr centroids_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

#endif  // TURTLEBOT3_BALL_COLLECTION__TSP_PLANNER_NODE_HPP_
