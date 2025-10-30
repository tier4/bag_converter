#include "nebula_decoder/nebula_decoder_node.hpp"

#include <pcl_conversions/pcl_conversions.h>

namespace nebula_decoder
{

NebulaDecoderNode::NebulaDecoderNode(const rclcpp::NodeOptions & options)
: Node("nebula_decoder", options)
{
  // Declare and get parameters
  sensor_model_ = this->declare_parameter<std::string>("sensor_model", "Robin_W");
  return_mode_ = this->declare_parameter<std::string>("return_mode", "Dual");
  frame_id_ = this->declare_parameter<std::string>("frame_id", "seyond");
  min_range_ = this->declare_parameter<double>("min_range", 0.3);
  max_range_ = this->declare_parameter<double>("max_range", 200.0);
  scan_phase_ = this->declare_parameter<double>("scan_phase", 0.0);
  frequency_ms_ = this->declare_parameter<double>("frequency_ms", 100.0);
  use_sensor_time_ = this->declare_parameter<bool>("use_sensor_time", true);

  // Create decoder configuration
  seyond_nebula_decoder::DecoderConfig config;
  config.sensor_model = sensor_model_;
  config.return_mode = return_mode_;
  config.frame_id = frame_id_;
  config.min_range = min_range_;
  config.max_range = max_range_;
  config.scan_phase = scan_phase_;
  config.frequency_ms = frequency_ms_;
  config.use_sensor_time = use_sensor_time_;

  // Initialize decoder
  decoder_ = std::make_unique<seyond_nebula_decoder::SeyondNebulaDecoder>(config);

  // Create publisher
  points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/nebula_points",
    rclcpp::QoS(10));

  // Create subscriber with sensor QoS
  packets_sub_ = this->create_subscription<nebula_msgs::msg::NebulaPackets>(
    "~/nebula_packets",
    rclcpp::SensorDataQoS(),
    std::bind(&NebulaDecoderNode::packetsCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Nebula Decoder Node initialized");
  RCLCPP_INFO(this->get_logger(), "  Sensor Model: %s", sensor_model_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Return Mode: %s", return_mode_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Frame ID: %s", frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Range: [%.2f, %.2f] m", min_range_, max_range_);
}

void NebulaDecoderNode::packetsCallback(const nebula_msgs::msg::NebulaPackets::SharedPtr msg)
{
  // Process each packet individually to handle scan completion
  for (const auto& packet : msg->packets) {
    auto [cloud, scan_complete] = decoder_->ProcessPacket(packet.data);

    // Publish when scan is complete
    if (scan_complete && cloud && !cloud->empty()) {
      // Convert to PointCloud2
      auto pc2_msg = convertToPointCloud2(cloud, msg->header);
      points_pub_->publish(pc2_msg);

      RCLCPP_DEBUG(
        this->get_logger(),
        "Published point cloud with %zu points",
        cloud->size());
    }
  }
}

sensor_msgs::msg::PointCloud2 NebulaDecoderNode::convertToPointCloud2(
  const nebula::drivers::NebulaPointCloudPtr& nebula_cloud,
  const std_msgs::msg::Header& header)
{
  sensor_msgs::msg::PointCloud2 output;

  // Use PCL conversion
  pcl::toROSMsg(*nebula_cloud, output);

  // Set header
  output.header = header;
  output.header.frame_id = frame_id_;

  return output;
}

}  // namespace nebula_decoder

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(nebula_decoder::NebulaDecoderNode)
