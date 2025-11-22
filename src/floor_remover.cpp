#include "D435i_floor_remover/floor_remover.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <cfloat>

namespace floor_remover
{

FloorRemover::FloorRemover()
{
  RCLCPP_INFO(logger_, "FloorRemover core initialized");
}

void FloorRemover::updateImu(const sensor_msgs::msg::Imu::SharedPtr& msg)
{
  imu_buffer_.push_back(msg);
  if (static_cast<int>(imu_buffer_.size()) > imu_buffer_size_) {
    imu_buffer_.pop_front();
  }
  updateGravityVector();
}

void FloorRemover::updateGravityVector()
{
  if (imu_buffer_.empty()) return;

  // Average acceleration over buffer to estimate gravity direction
  Eigen::Vector3d acc_sum(0.0, 0.0, 0.0);

  for (const auto& imu_msg : imu_buffer_) {
    acc_sum.x() += imu_msg->linear_acceleration.x;
    acc_sum.y() += imu_msg->linear_acceleration.y;
    acc_sum.z() += imu_msg->linear_acceleration.z;
  }
  acc_sum /= static_cast<double>(imu_buffer_.size());

  // IMU measures acceleration opposite to gravity when stationary
  if (acc_sum.norm() > 0.1) {
    Eigen::Vector3d new_gravity = -acc_sum.normalized();

    // Apply low-pass filter to smooth gravity estimate
    Eigen::Vector3d filtered_gravity = gravity_filter_alpha_ * new_gravity + (1.0 - gravity_filter_alpha_) * gravity_vector_;
    filtered_gravity.normalize();

    gravity_vector_ = filtered_gravity;
  }
}

bool FloorRemover::processPointCloud(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud_no_floor,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_floor_only,
  Eigen::Vector4d& plane_coefficients
)
{
  if (input_cloud->points.empty()) {
    return false;
  }

  // Step 1: Clean and filter by distance
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clean(new pcl::PointCloud<pcl::PointXYZRGB>);

  for (const auto& point : input_cloud->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

    float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (distance < min_distance_ || distance > max_distance_) {
      continue;
    }

    cloud_clean->points.push_back(point);
  }

  cloud_clean->width = cloud_clean->points.size();
  cloud_clean->height = 1;
  cloud_clean->is_dense = false;

  // Step 2: Voxel grid downsampling
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
  voxel_filter.setInputCloud(cloud_clean);
  voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  voxel_filter.filter(*cloud_filtered);

  // Step 3: Find floor plane
  Eigen::Vector4d plane_coeffs;
  bool plane_found = findFloorPlane(cloud_filtered, plane_coeffs);

  Eigen::Vector4d plane_to_use;
  bool use_plane = false;

  if (plane_found) {
    plane_to_use = plane_coeffs;
    previous_plane_coeffs_ = plane_coeffs;
    has_previous_plane_ = true;
    use_plane = true;
  } else if (has_previous_plane_) {
    plane_to_use = previous_plane_coeffs_;
    use_plane = true;
  }

  if (use_plane) {
    removeFloorPlane(cloud_filtered, plane_to_use, output_cloud_no_floor, output_floor_only);
    plane_coefficients = plane_to_use;
    return true;
  }

  return false;
}

bool FloorRemover::findFloorPlane(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
  Eigen::Vector4d& plane_coeffs
)
{
  if (cloud->points.size() < 3) {
    return false;
  }

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(plane_max_iterations_);
  seg.setDistanceThreshold(plane_distance_threshold_);

  // Use PERPENDICULAR_PLANE model to find plane perpendicular to gravity
  if (use_imu_for_validation_ && !imu_buffer_.empty()) {
    Eigen::Vector3d floor_axis = -gravity_vector_;
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setAxis(Eigen::Vector3f(floor_axis.x(), floor_axis.y(), floor_axis.z()));
    seg.setEpsAngle(0.17);  // ~10 degrees
  } else {
    Eigen::Vector3f fixed_axis(0.0, -1.0, 0.0);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setAxis(fixed_axis);
    seg.setEpsAngle(0.17);  // ~10 degrees
  }

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
    return false;
  }

  // Extract plane normal
  Eigen::Vector3d plane_normal(
    coefficients->values[0],
    coefficients->values[1],
    coefficients->values[2]
  );
  plane_normal.normalize();

  // Validation: Check against gravity vector
  if (use_imu_for_validation_ && !imu_buffer_.empty()) {
    Eigen::Vector3d imu_floor_normal = -gravity_vector_;
    double imu_alignment = plane_normal.dot(imu_floor_normal);

    if (imu_alignment < floor_normal_alignment_threshold_) {
      return false;
    }
  } else {
    Eigen::Vector3d fixed_floor_normal(0.0, -1.0, 0.0);
    double alignment = plane_normal.dot(fixed_floor_normal);

    if (alignment < floor_normal_alignment_threshold_) {
      return false;
    }
  }

  // Height validation
  double plane_height = -coefficients->values[3];
  if (plane_height > max_height_threshold_ || plane_height < -0.5) {
    return false;
  }

  // Store plane coefficients
  plane_coeffs << coefficients->values[0],
                  coefficients->values[1],
                  coefficients->values[2],
                  coefficients->values[3];

  return true;
}

void FloorRemover::removeFloorPlane(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in,
  const Eigen::Vector4d& plane_coeffs,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& floor_out
)
{
  cloud_out->points.clear();
  floor_out->points.clear();

  for (const auto& point : cloud_in->points) {
    // Calculate distance from point to plane
    double distance = plane_coeffs[0] * point.x +
                      plane_coeffs[1] * point.y +
                      plane_coeffs[2] * point.z +
                      plane_coeffs[3];

    if (std::abs(distance) < plane_distance_threshold_) {
      floor_out->points.push_back(point);
    } else if (distance > 0 && distance < max_height_threshold_) {
      cloud_out->points.push_back(point);
    }
  }

  cloud_out->width = cloud_out->points.size();
  cloud_out->height = 1;
  cloud_out->is_dense = false;

  floor_out->width = floor_out->points.size();
  floor_out->height = 1;
  floor_out->is_dense = false;
}

} // namespace floor_remover
