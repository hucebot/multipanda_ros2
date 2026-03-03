#ifndef BOTA_DRIVER_NODE_HPP
#define BOTA_DRIVER_NODE_HPP

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "bota_driver/msg/bota_frame.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "bota_driver_exposed.hpp"

class BotaDriverNode : public rclcpp::Node
{
public:
  BotaDriverNode();
  explicit BotaDriverNode(const std::string& node_name);
  ~BotaDriverNode();

private:
  // Publishes the data from a frame reading
  void publish_frame_data(const bota::BotaFrame& frame_reading);
  
  // Reads a frame synchronously and publishes it
  void read_sync_and_publish();
  
  // Methods for asynchronous reading
  void start_async_reading();
  void async_reading_loop();
  void stop_async_reading();

  // Service callbacks
  void tare_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Configuration file path
  std::string config_file_;

  // Driver object
  std::unique_ptr<bota::BotaDriverExposed> driver_;

  // ROS2 publishers
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_bota_wrench_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_bota_imu_;
  rclcpp::Publisher<bota_driver::msg::BotaFrame>::SharedPtr publisher_bota_frame_;

  // ROS2 service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tare_service_;

  // For synchronous reading
  float output_rate_;
  rclcpp::TimerBase::SharedPtr timer_;

  // For asynchronous reading
  std::thread async_thread_;
  std::atomic<bool> async_thread_running_{false};

  // Node name used for topic and service prefixes
  std::string bota_driver_node_name_;
};

#endif // BOTA_DRIVER_NODE_HPP
