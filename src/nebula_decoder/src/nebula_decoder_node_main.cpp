#include "nebula_decoder/nebula_decoder_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<nebula_decoder::NebulaDecoderNode>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
