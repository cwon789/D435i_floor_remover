#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
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

    // Create publishers
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 10);
    floor_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(floor_cloud_topic_, 10);
    gravity_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(gravity_marker_topic_, 10);

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
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    floor_remover_->updateImu(msg);
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

  // Floor remover core
  std::unique_ptr<floor_remover::FloorRemover> floor_remover_;

  // ROS communication
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr floor_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr gravity_marker_pub_;

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
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FloorRemoverNode>());
  rclcpp::shutdown();
  return 0;
}
