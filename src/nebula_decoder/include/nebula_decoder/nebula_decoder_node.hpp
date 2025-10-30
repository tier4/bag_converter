#ifndef NEBULA_DECODER_NODE_HPP
#define NEBULA_DECODER_NODE_HPP

#include "seyond_nebula_decoder/seyond_nebula_decoder.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <memory>
#include <string>

namespace nebula_decoder
{

class NebulaDecoderNode : public rclcpp::Node
{
public:
  explicit NebulaDecoderNode(const rclcpp::NodeOptions & options);
  ~NebulaDecoderNode() = default;

private:
  void packetsCallback(const nebula_msgs::msg::NebulaPackets::SharedPtr msg);

  sensor_msgs::msg::PointCloud2 convertToPointCloud2(
    const nebula::drivers::NebulaPointCloudPtr& nebula_cloud,
    const std_msgs::msg::Header& header);

  // Decoder
  std::unique_ptr<seyond_nebula_decoder::SeyondNebulaDecoder> decoder_;

  // ROS interfaces
  rclcpp::Subscription<nebula_msgs::msg::NebulaPackets>::SharedPtr packets_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;

  // Parameters
  std::string sensor_model_;
  std::string return_mode_;
  std::string frame_id_;
  double min_range_;
  double max_range_;
  double scan_phase_;
  double frequency_ms_;
  bool use_sensor_time_;
};

}  // namespace nebula_decoder

#endif  // NEBULA_DECODER_NODE_HPP
