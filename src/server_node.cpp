#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "D435i_floor_remover/floor_remover.hpp"

class FloorRemoverNode : public rclcpp::Node
{
public:
  FloorRemoverNode() : Node("floor_remover_node")
  {
    // Declare and get parameters
    declareParameters();
    loadParameters();

    // Create floor remover instance
    floor_remover_ = std::make_unique<floor_remover::FloorRemover>();
    configureFloorRemover();

    // Create subscribers
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_, 10,
      std::bind(&FloorRemoverNode::cloudCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      input_imu_topic_, 10,
      std::bind(&FloorRemoverNode::imuCallback, this, std::placeholders::_1));

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/depth/camera_info", 10,
      std::bind(&FloorRemoverNode::cameraInfoCallback, this, std::placeholders::_1));

    // Create publishers
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 10);
    floor_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(floor_cloud_topic_, 10);
    gravity_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(gravity_marker_topic_, 10);

    floor_depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/floor_depth", 10);
    floor_removed_depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/floor_removed_depth", 10);

    RCLCPP_INFO(this->get_logger(), "Floor Remover Node initialized");
    RCLCPP_INFO(this->get_logger(), "Input cloud: %s", input_cloud_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Input IMU: %s", input_imu_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Use IMU validation: %s", use_imu_for_validation_ ? "true" : "false");
  }

private:
  void declareParameters()
  {
    this->declare_parameter("input_cloud_topic", "/camera/depth/color/points");
    this->declare_parameter("input_imu_topic", "/camera/imu");
    this->declare_parameter("output_cloud_topic", "/floor_removed_cloud");
    this->declare_parameter("floor_cloud_topic", "/floor_cloud");
    this->declare_parameter("gravity_marker_topic", "/gravity_vector");
    this->declare_parameter("voxel_leaf_size", 0.01);
    this->declare_parameter("plane_distance_threshold", 0.02);
    this->declare_parameter("plane_max_iterations", 100);
    this->declare_parameter("imu_buffer_size", 50);
    this->declare_parameter("use_imu_for_validation", true);
    this->declare_parameter("max_height_threshold", 2.0);
    this->declare_parameter("floor_normal_alignment_threshold", 0.7);
    this->declare_parameter("min_distance", 0.3);
    this->declare_parameter("max_distance", 5.0);
    this->declare_parameter("gravity_filter_alpha", 0.3);
    this->declare_parameter("depth_image_projection", true);
    this->declare_parameter("depth_interpolation_iterations", 3);
    this->declare_parameter("min_neighbors_for_interpolation", 2);
    this->declare_parameter("additional_removal_margin", 0.0);
  }

  void loadParameters()
  {
    input_cloud_topic_ = this->get_parameter("input_cloud_topic").as_string();
    input_imu_topic_ = this->get_parameter("input_imu_topic").as_string();
    output_cloud_topic_ = this->get_parameter("output_cloud_topic").as_string();
    floor_cloud_topic_ = this->get_parameter("floor_cloud_topic").as_string();
    gravity_marker_topic_ = this->get_parameter("gravity_marker_topic").as_string();
    voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
    plane_distance_threshold_ = this->get_parameter("plane_distance_threshold").as_double();
    plane_max_iterations_ = this->get_parameter("plane_max_iterations").as_int();
    imu_buffer_size_ = this->get_parameter("imu_buffer_size").as_int();
    use_imu_for_validation_ = this->get_parameter("use_imu_for_validation").as_bool();
    max_height_threshold_ = this->get_parameter("max_height_threshold").as_double();
    floor_normal_alignment_threshold_ = this->get_parameter("floor_normal_alignment_threshold").as_double();
    min_distance_ = this->get_parameter("min_distance").as_double();
    max_distance_ = this->get_parameter("max_distance").as_double();
    gravity_filter_alpha_ = this->get_parameter("gravity_filter_alpha").as_double();
    depth_image_projection_ = this->get_parameter("depth_image_projection").as_bool();
    depth_interpolation_iterations_ = this->get_parameter("depth_interpolation_iterations").as_int();
    min_neighbors_for_interpolation_ = this->get_parameter("min_neighbors_for_interpolation").as_int();
    additional_removal_margin_ = this->get_parameter("additional_removal_margin").as_double();
  }

  void configureFloorRemover()
  {
    floor_remover_->setVoxelLeafSize(voxel_leaf_size_);
    floor_remover_->setPlaneDistanceThreshold(plane_distance_threshold_);
    floor_remover_->setPlaneMaxIterations(plane_max_iterations_);
    floor_remover_->setMinDistance(min_distance_);
    floor_remover_->setMaxDistance(max_distance_);
    floor_remover_->setMaxHeightThreshold(max_height_threshold_);
    floor_remover_->setFloorNormalAlignmentThreshold(floor_normal_alignment_threshold_);
    floor_remover_->setUseImuForValidation(use_imu_for_validation_);
    floor_remover_->setImuBufferSize(imu_buffer_size_);
    floor_remover_->setGravityFilterAlpha(gravity_filter_alpha_);
    floor_remover_->setAdditionalRemovalMargin(additional_removal_margin_);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    floor_remover_->updateImu(msg);
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (!camera_info_received_) {
      camera_info_ = msg;
      camera_info_received_ = true;
      RCLCPP_INFO(this->get_logger(), "Camera info received: %dx%d", msg->width, msg->height);
    }
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convert ROS message to PCL
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->points.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
      return;
    }

    // Process point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_no_floor(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_floor(new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Vector4d plane_coeffs;

    bool success = floor_remover_->processPointCloud(cloud, cloud_no_floor, cloud_floor, plane_coeffs);

    if (success) {
      RCLCPP_INFO(this->get_logger(), "✓ Floor plane [%.3f, %.3f, %.3f, %.3f]",
                  plane_coeffs[0], plane_coeffs[1], plane_coeffs[2], plane_coeffs[3]);
    } else {
      RCLCPP_WARN(this->get_logger(), "✗ No floor plane found");
    }

    // Publish results
    publishCloud(cloud_pub_, cloud_no_floor, msg->header);
    publishCloud(floor_pub_, cloud_floor, msg->header);
    publishGravityMarker(msg->header);

    // Publish depth images from point clouds (only if enabled)
    if (depth_image_projection_ && camera_info_received_) {
      publishDepthImages(cloud_floor, cloud_no_floor, msg->header);
    }
  }

  void publishCloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const std_msgs::msg::Header& header)
  {
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header = header;
    publisher->publish(output);
  }

  void publishGravityMarker(const std_msgs::msg::Header& header)
  {
    Eigen::Vector3d gravity = floor_remover_->getGravityVector();

    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "gravity";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;

    marker.scale.x = 0.02;  // 화살표 길이
    marker.scale.y = 0.003; // 화살표 너비
    marker.scale.z = 0.003; // 화살표 높이

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    geometry_msgs::msg::Point start, end;
    start.x = 0.0;
    start.y = 0.0;
    start.z = 0.0;
    end.x = gravity.x();
    end.y = gravity.y();
    end.z = gravity.z();

    marker.points.push_back(start);
    marker.points.push_back(end);

    gravity_marker_pub_->publish(marker);
  }

  void publishDepthImages(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& floor_cloud,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& no_floor_cloud,
    const std_msgs::msg::Header& header)
  {
    if (!camera_info_received_) return;

    const int width = camera_info_->width;
    const int height = camera_info_->height;

    // Get camera intrinsics
    const double fx = camera_info_->k[0];
    const double fy = camera_info_->k[4];
    const double cx = camera_info_->k[2];
    const double cy = camera_info_->k[5];

    // Create depth images (16-bit, mm units)
    std::vector<uint16_t> floor_depth(width * height, 0);
    std::vector<uint16_t> floor_removed_depth(width * height, 0);

    // Project floor points to depth image
    for (const auto& point : floor_cloud->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
        continue;
      if (point.z <= 0.0) continue;

      int u = static_cast<int>(fx * point.x / point.z + cx);
      int v = static_cast<int>(fy * point.y / point.z + cy);

      if (u >= 0 && u < width && v >= 0 && v < height) {
        uint16_t depth_mm = static_cast<uint16_t>(point.z * 1000.0);
        floor_depth[v * width + u] = depth_mm;
      }
    }

    // Project no-floor points to depth image
    for (const auto& point : no_floor_cloud->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
        continue;
      if (point.z <= 0.0) continue;

      int u = static_cast<int>(fx * point.x / point.z + cx);
      int v = static_cast<int>(fy * point.y / point.z + cy);

      if (u >= 0 && u < width && v >= 0 && v < height) {
        uint16_t depth_mm = static_cast<uint16_t>(point.z * 1000.0);
        floor_removed_depth[v * width + u] = depth_mm;
      }
    }

    // Apply interpolation multiple times to fill holes
    for (int iter = 0; iter < depth_interpolation_iterations_; ++iter) {
      interpolateDepth(floor_depth, width, height, min_neighbors_for_interpolation_);
      interpolateDepth(floor_removed_depth, width, height, min_neighbors_for_interpolation_);
    }

    // Convert to ROS messages
    sensor_msgs::msg::Image floor_depth_msg;
    sensor_msgs::msg::Image floor_removed_depth_msg;

    floor_depth_msg.header = header;
    floor_depth_msg.height = height;
    floor_depth_msg.width = width;
    floor_depth_msg.encoding = "16UC1";
    floor_depth_msg.is_bigendian = 0;
    floor_depth_msg.step = width * 2;
    floor_depth_msg.data.resize(width * height * 2);

    floor_removed_depth_msg.header = header;
    floor_removed_depth_msg.height = height;
    floor_removed_depth_msg.width = width;
    floor_removed_depth_msg.encoding = "16UC1";
    floor_removed_depth_msg.is_bigendian = 0;
    floor_removed_depth_msg.step = width * 2;
    floor_removed_depth_msg.data.resize(width * height * 2);

    for (size_t i = 0; i < static_cast<size_t>(width * height); ++i) {
      floor_depth_msg.data[i * 2] = floor_depth[i] & 0xFF;
      floor_depth_msg.data[i * 2 + 1] = (floor_depth[i] >> 8) & 0xFF;

      floor_removed_depth_msg.data[i * 2] = floor_removed_depth[i] & 0xFF;
      floor_removed_depth_msg.data[i * 2 + 1] = (floor_removed_depth[i] >> 8) & 0xFF;
    }

    floor_depth_pub_->publish(floor_depth_msg);
    floor_removed_depth_pub_->publish(floor_removed_depth_msg);
  }

  void interpolateDepth(std::vector<uint16_t>& depth, int width, int height, int min_neighbors)
  {
    // Fast conditional interpolation with 3x3 kernel
    // Only interpolate pixels with at least min_neighbors valid neighbors
    std::vector<uint16_t> output = depth;

    for (int v = 1; v < height - 1; ++v) {  // Skip borders for speed
      for (int u = 1; u < width - 1; ++u) {
        int idx = v * width + u;

        if (depth[idx] == 0) {
          // Check 3x3 neighborhood
          int sum = 0;
          int count = 0;

          // Unrolled 3x3 kernel for speed
          int nw = depth[(v-1) * width + (u-1)];
          int n  = depth[(v-1) * width + u];
          int ne = depth[(v-1) * width + (u+1)];
          int w  = depth[v * width + (u-1)];
          int e  = depth[v * width + (u+1)];
          int sw = depth[(v+1) * width + (u-1)];
          int s  = depth[(v+1) * width + u];
          int se = depth[(v+1) * width + (u+1)];

          if (nw > 0) { sum += nw; count++; }
          if (n  > 0) { sum += n;  count++; }
          if (ne > 0) { sum += ne; count++; }
          if (w  > 0) { sum += w;  count++; }
          if (e  > 0) { sum += e;  count++; }
          if (sw > 0) { sum += sw; count++; }
          if (s  > 0) { sum += s;  count++; }
          if (se > 0) { sum += se; count++; }

          // Only interpolate if we have at least min_neighbors
          if (count >= min_neighbors) {
            output[idx] = sum / count;
          }
        }
      }
    }

    depth = output;
  }

  // Floor remover core
  std::unique_ptr<floor_remover::FloorRemover> floor_remover_;

  // ROS communication
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr floor_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr gravity_marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr floor_depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr floor_removed_depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr floor_depth_color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr floor_removed_depth_color_pub_;

  // Camera info
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
  bool camera_info_received_ = false;

  // Parameters
  std::string input_cloud_topic_;
  std::string input_imu_topic_;
  std::string output_cloud_topic_;
  std::string floor_cloud_topic_;
  std::string gravity_marker_topic_;
  double voxel_leaf_size_;
  double plane_distance_threshold_;
  int plane_max_iterations_;
  int imu_buffer_size_;
  bool use_imu_for_validation_;
  double max_height_threshold_;
  double floor_normal_alignment_threshold_;
  double min_distance_;
  double max_distance_;
  double gravity_filter_alpha_;
  bool depth_image_projection_;
  int depth_interpolation_iterations_;
  int min_neighbors_for_interpolation_;
  double additional_removal_margin_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FloorRemoverNode>());
  rclcpp::shutdown();
  return 0;
}
