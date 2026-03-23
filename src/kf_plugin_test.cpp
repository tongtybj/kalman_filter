#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <kalman_filter/kf_base_plugin.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("kf_plugin_test");

  pluginlib::ClassLoader<kf_plugin::KalmanFilter> kf_loader("kalman_filter", "kf_plugin::KalmanFilter");

  try {
    auto kf_pos_vel_acc = kf_loader.createSharedInstance("kalman_filter/kf_pos_vel_acc");
    kf_pos_vel_acc->initialize("test", 0);
    RCLCPP_INFO(node->get_logger(), "Plugin loaded successfully");
  } catch (const pluginlib::PluginlibException& ex) {
    RCLCPP_ERROR(node->get_logger(),
      "The plugin failed to load for some reason. Error: %s", ex.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
