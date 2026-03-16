#ifndef TURTLEBOT3_BALL_COLLECTION__CLUSTERING_NODE_HPP_
#define TURTLEBOT3_BALL_COLLECTION__CLUSTERING_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class ClusteringNode : public rclcpp::Node
{
public:
  ClusteringNode();

private:
  void ball_positions_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  std::vector<geometry_msgs::msg::Point> dbscan_clustering(const std::vector<geometry_msgs::msg::Point>& points);
  void publish_centroids(const std::vector<geometry_msgs::msg::Point>& centroids);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ball_positions_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr centroids_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  double eps_;
  int min_samples_;
};

#endif  // TURTLEBOT3_BALL_COLLECTION__CLUSTERING_NODE_HPP_
