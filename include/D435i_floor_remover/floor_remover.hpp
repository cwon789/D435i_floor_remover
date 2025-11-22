#ifndef FLOOR_REMOVER_HPP
#define FLOOR_REMOVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>

#include <deque>
#include <memory>

namespace floor_remover
{

class FloorRemover
{
public:
  FloorRemover();
  ~FloorRemover() = default;

  // Configuration
  void setVoxelLeafSize(double size) { voxel_leaf_size_ = size; }
  void setPlaneDistanceThreshold(double threshold) { plane_distance_threshold_ = threshold; }
  void setPlaneMaxIterations(int iterations) { plane_max_iterations_ = iterations; }
  void setMinDistance(double distance) { min_distance_ = distance; }
  void setMaxDistance(double distance) { max_distance_ = distance; }
  void setMaxHeightThreshold(double height) { max_height_threshold_ = height; }
  void setFloorNormalAlignmentThreshold(double threshold) { floor_normal_alignment_threshold_ = threshold; }
  void setUseImuForValidation(bool use) { use_imu_for_validation_ = use; }
  void setImuBufferSize(int size) { imu_buffer_size_ = size; }
  void setGravityFilterAlpha(double alpha) { gravity_filter_alpha_ = alpha; }

  // IMU update
  void updateImu(const sensor_msgs::msg::Imu::SharedPtr& msg);
  Eigen::Vector3d getGravityVector() const { return gravity_vector_; }

  // Point cloud processing
  bool processPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud_no_floor,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_floor_only,
    Eigen::Vector4d& plane_coefficients
  );

private:
  // Core processing functions
  bool findFloorPlane(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    Eigen::Vector4d& plane_coeffs
  );

  void removeFloorPlane(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in,
    const Eigen::Vector4d& plane_coeffs,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& floor_out
  );

  void updateGravityVector();

  // Parameters
  double voxel_leaf_size_{0.01};
  double plane_distance_threshold_{0.02};
  int plane_max_iterations_{100};
  double min_distance_{0.3};
  double max_distance_{5.0};
  double max_height_threshold_{2.0};
  double floor_normal_alignment_threshold_{0.7};
  bool use_imu_for_validation_{true};
  int imu_buffer_size_{50};
  double gravity_filter_alpha_{0.3};

  // State
  std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer_;
  Eigen::Vector3d gravity_vector_{0.0, 1.0, 0.0};
  Eigen::Vector4d previous_plane_coeffs_{Eigen::Vector4d::Zero()};
  bool has_previous_plane_{false};

  // Logger (optional, for debugging)
  rclcpp::Logger logger_{rclcpp::get_logger("floor_remover")};
};

} // namespace floor_remover

#endif // FLOOR_REMOVER_HPP
