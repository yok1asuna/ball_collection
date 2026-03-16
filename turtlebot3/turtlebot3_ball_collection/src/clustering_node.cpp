#include "clustering_node.hpp"

#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

ClusteringNode::ClusteringNode() : Node("clustering_node")
{
  ball_positions_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "ball_positions", 10,
    std::bind(&ClusteringNode::ball_positions_callback, this, std::placeholders::_1));

  centroids_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("centroids", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("centroid_markers", 10);

  this->declare_parameter<double>("eps", 1.0);
  this->declare_parameter<int>("min_samples", 2);

  this->get_parameter("eps", eps_);
  this->get_parameter("min_samples", min_samples_);
}

void ClusteringNode::ball_positions_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(eps_);
  ec.setMinClusterSize(min_samples_);
  ec.setMaxClusterSize(100);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  std::vector<geometry_msgs::msg::Point> centroids;
  for (const auto& indices : cluster_indices) {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, indices, centroid);
    geometry_msgs::msg::Point point;
    point.x = centroid[0];
    point.y = centroid[1];
    point.z = centroid[2];
    centroids.push_back(point);
  }

  publish_centroids(centroids);
}

void ClusteringNode::publish_centroids(const std::vector<geometry_msgs::msg::Point>& centroids)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  visualization_msgs::msg::MarkerArray markers;

  for (size_t i = 0; i < centroids.size(); ++i) {
    pcl::PointXYZ point(centroids[i].x, centroids[i].y, centroids[i].z);
    cloud.points.push_back(point);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "centroids";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = centroids[i];
    marker.scale.x = marker.scale.y = marker.scale.z = 0.2;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    markers.markers.push_back(marker);
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = "map";
  cloud_msg.header.stamp = this->now();

  centroids_pub_->publish(cloud_msg);
  marker_pub_->publish(markers);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClusteringNode>());
  rclcpp::shutdown();
  return 0;
}
