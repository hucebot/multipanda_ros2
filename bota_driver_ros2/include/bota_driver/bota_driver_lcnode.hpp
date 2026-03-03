#ifndef BOTA_DRIVER_LCNODE_HPP
#define BOTA_DRIVER_LCNODE_HPP

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "bota_driver/msg/bota_frame.hpp"

#include "bota_driver_exposed.hpp"

#include "std_srvs/srv/trigger.hpp"

class BotaDriverLCNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  BotaDriverLCNode();
  ~BotaDriverLCNode();

  // Lifecycle callbacks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_configure(const rclcpp_lifecycle::State &);
  
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_cleanup(const rclcpp_lifecycle::State &);
  
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_activate(const rclcpp_lifecycle::State &);
  
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_deactivate(const rclcpp_lifecycle::State &);
  
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_shutdown(const rclcpp_lifecycle::State &);
  
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_error(const rclcpp_lifecycle::State &);

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

  // Configuration
  std::string config_file_;
  double output_rate_;

  // Driver object
  std::unique_ptr<bota::BotaDriverExposed> driver_;

  // ROS2 publishers
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_bota_wrench_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr publisher_bota_imu_;
  rclcpp_lifecycle::LifecyclePublisher<bota_driver::msg::BotaFrame>::SharedPtr publisher_bota_frame_;

    // ROS2 service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tare_service_;  

  // For synchronous reading
  rclcpp::TimerBase::SharedPtr timer_;

  // For asynchronous reading
  std::thread async_thread_;
  std::atomic<bool> async_thread_running_{false};

  // Frame ID for the topic
  std::string bota_ft_sensor_link_name_;
};

#endif // BOTA_DRIVER_LCNODE_HPP