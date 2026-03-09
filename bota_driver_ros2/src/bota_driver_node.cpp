#include "bota_driver/bota_driver_node.hpp"
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "bota_driver/msg/bota_frame.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

BotaDriverNode::BotaDriverNode(const std::string &node_name)
    : Node(node_name), bota_driver_node_name_(node_name)
{
  // Get mandatory config file parameter
  try
  {
    config_file_ = declare_parameter<std::string>("config_file");
    RCLCPP_INFO(get_logger(), "Using config file: '%s'", config_file_.c_str());
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(get_logger(),
                 "Required parameter 'config_file' was not provided. "
                 "Please set it in your launch file or via --ros-args -p config_file:=<path>");
    rclcpp::shutdown();
    return;
  }

  // Get mandatory output_rate parameter
  try
  {
    // Try to declare it as a double
    output_rate_ = declare_parameter<float>("output_rate");
    RCLCPP_INFO(get_logger(), "Using output rate: %.2f Hz", output_rate_);
  }
  catch (const rclcpp::exceptions::InvalidParameterTypeException &e)
  {
    // They passed an integer → redeclare as int to pick it up
    RCLCPP_WARN(get_logger(),
                "Parameter 'output_rate' was provided as integer; converting to float");
    try
    {
      int tmp = declare_parameter<int>("output_rate");
      output_rate_ = static_cast<float>(tmp);
    }
    catch (const std::exception &e2)
    {
      // Even the int‐declare failed (i.e. no override at all)
      RCLCPP_FATAL(get_logger(),
                   "Required parameter 'output_rate' was not provided: %s",
                   e2.what());
      rclcpp::shutdown();
      return;
    }
  }
  catch (const std::exception &e)
  {
    // Any other problem (missing entirely, bad name, etc) → fatal
    RCLCPP_FATAL(get_logger(),
                 "Required parameter 'output_rate' was not provided or is invalid: %s",
                 e.what());
    rclcpp::shutdown();
    return;
  }

  // Initialize driver with the user-provided config file
  try
  {
    driver_ = std::make_unique<bota::BotaDriverExposed>(config_file_);
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(get_logger(),
                 "Failed to initialize BotaDriverExposed with config file '%s': %s",
                 config_file_.c_str(), e.what());
    rclcpp::shutdown();
    return;
  }

  // Set up publishers with prefixed topic names
  publisher_bota_wrench_ =
      create_publisher<geometry_msgs::msg::WrenchStamped>(bota_driver_node_name_ + "/wrench", 10);
  publisher_bota_imu_ =
      create_publisher<sensor_msgs::msg::Imu>(bota_driver_node_name_ + "/imu", 10);
  publisher_bota_frame_ =
      create_publisher<bota_driver::msg::BotaFrame>(bota_driver_node_name_ + "/bota_frame", 10);

  // Add tare service with prefixed name
  tare_service_ = create_service<std_srvs::srv::Trigger>(
      bota_driver_node_name_ + "/tare", std::bind(&BotaDriverNode::tare_callback, this,
                                                     std::placeholders::_1, std::placeholders::_2));

  driver_->onConfigure();
  driver_->onActivate();

  // Sync vs Async reading setup
  if (output_rate_ < 0.0)
  {
    RCLCPP_FATAL(get_logger(),
                 "output_rate must be >= 0 (0: rate set by the stream, >0: timed polling from the buffer))");
    rclcpp::shutdown();
  }
  else if (output_rate_ == 0.0)
  {
    RCLCPP_INFO(get_logger(),
                "Async reading: topic rate set by sensor stream");
    start_async_reading();
  }
  else
  {
    RCLCPP_INFO(get_logger(),
                "Sync reading: polling data from the buffer and publishing at %.2f Hz", output_rate_);
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / output_rate_));
    timer_ = create_wall_timer(period, std::bind(&BotaDriverNode::read_sync_and_publish, this));
  }

  RCLCPP_INFO(get_logger(), "BotaDriverNode initialized successfully");
}

BotaDriverNode::~BotaDriverNode()
{
  RCLCPP_INFO(get_logger(), "bota_driver_node shutting down");
  stop_async_reading();
  driver_->onDeactivate();
  driver_->onCleanup();
  driver_->onShutdown();
}

void BotaDriverNode::publish_frame_data(const bota::BotaFrame &frame)
{
  // Protect from race conditions
  std::lock_guard<std::mutex> lock(filter_mutex_);

  // Apply low-pass filtering
  for (int i = 0; i < 3; ++i)
  {
    fil_sensor_wrench_[i] = sensor_wrench_filter_alpha_ * frame.force[i] + (1 - sensor_wrench_filter_alpha_) * fil_sensor_wrench_[i];
  }
  for (int i = 0; i < 3; ++i)
  {
    fil_sensor_wrench_[i+3] = sensor_wrench_filter_alpha_ * frame.torque[i] + (1 - sensor_wrench_filter_alpha_) * fil_sensor_wrench_[i+3];
  }

  // Get the current time so the topics can be stamped with the same time
  rclcpp::Time stamp = now();
  
  // Wrench
  geometry_msgs::msg::WrenchStamped wrench_msg;
  wrench_msg.header.stamp = stamp;
  wrench_msg.header.frame_id = bota_driver_node_name_ + "_wrench";
  wrench_msg.wrench.force.x = fil_sensor_wrench_[0];
  wrench_msg.wrench.force.y = fil_sensor_wrench_[1];
  wrench_msg.wrench.force.z = fil_sensor_wrench_[2];
  wrench_msg.wrench.torque.x = fil_sensor_wrench_[3];
  wrench_msg.wrench.torque.y = fil_sensor_wrench_[4];
  wrench_msg.wrench.torque.z = fil_sensor_wrench_[5];
  publisher_bota_wrench_->publish(wrench_msg);

  // Imu
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = stamp;
  imu_msg.header.frame_id = bota_driver_node_name_ + "_imu";
  imu_msg.orientation.x = 0.0; // Assuming no orientation data available
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0; // Default orientation (no rotation)
  imu_msg.angular_velocity.x = frame.angular_rate[0];
  imu_msg.angular_velocity.y = frame.angular_rate[1];
  imu_msg.angular_velocity.z = frame.angular_rate[2];
  imu_msg.linear_acceleration.x = frame.acceleration[0];
  imu_msg.linear_acceleration.y = frame.acceleration[1];
  imu_msg.linear_acceleration.z = frame.acceleration[2];
  imu_msg.orientation_covariance.fill(0.0);
  imu_msg.angular_velocity_covariance.fill(0.0);
  imu_msg.linear_acceleration_covariance.fill(0.0);
  publisher_bota_imu_->publish(imu_msg);

  // Full frame
  bota_driver::msg::BotaFrame bf_msg;

  bf_msg.throttled = frame.status.bits.throttled;
  bf_msg.overrange = frame.status.bits.overrange;
  bf_msg.invalid = frame.status.bits.invalid;
  bf_msg.raw = frame.status.bits.raw;

  bf_msg.wrench = wrench_msg;
  bf_msg.imu = imu_msg;
  bf_msg.temperature = frame.temperature;
  bf_msg.timestamp = frame.timestamp;

  publisher_bota_frame_->publish(bf_msg);
}

void BotaDriverNode::read_sync_and_publish()
{
  try
  {
    auto frame = driver_->readFrame();
    publish_frame_data(frame);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(get_logger(), "Error reading sync: %s", e.what());
    driver_->onError();
  }
}

void BotaDriverNode::start_async_reading()
{
  async_thread_running_ = true;
  async_thread_ = std::thread(&BotaDriverNode::async_reading_loop, this);
}

void BotaDriverNode::async_reading_loop()
{
  while (async_thread_running_ && rclcpp::ok())
  {
    try
    {
      auto frame = driver_->readFrameBlocking();
      publish_frame_data(frame);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Error in async read: %s", e.what());
      driver_->onError();
      std::this_thread::sleep_for(100ms);
    }
  }
}

void BotaDriverNode::stop_async_reading()
{
  async_thread_running_ = false;
  if (async_thread_.joinable())
  {
    async_thread_.join();
  }
}

void BotaDriverNode::tare_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request; // Mark as unused

  try
  {
    // Stop the async reading if it's running
    if (async_thread_running_)
    {
      stop_async_reading();
    }

    // Bring the driver from ACTIVE to INACTIVE state
    driver_->onDeactivate();

    // Call the tare method
    if (driver_->tare())
    {
      std::lock_guard<std::mutex> lock(filter_mutex_);
      fil_sensor_wrench_.fill(0.0); // Reset filter
      RCLCPP_INFO(get_logger(), "Sensor tared successfully");
      response->success = true;
      response->message = "Sensor tared successfully";
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Tare operation failed");
      response->success = false;
      response->message = "Tare operation failed";
    }

    // Bring the driver from INACTIVE to ACTIVE state
    driver_->onActivate();

    // Start async reading again
    if (output_rate_ == 0.0)
    {
      start_async_reading();
    }
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(get_logger(), "Failed to tare sensor: %s", e.what());
    response->success = false;
    response->message = "Failed to tare sensor: " + std::string(e.what());
  }
}

// Factory function to create node with dynamic name
std::shared_ptr<BotaDriverNode> create_bota_driver_node()
{
  // Create a temporary node to get the parameter
  auto temp_node = rclcpp::Node::make_shared("temp_bota_driver");

  std::string node_name;
  try
  {
    node_name = temp_node->declare_parameter<std::string>("node_name");
    RCLCPP_INFO(temp_node->get_logger(), "Using provided node name: '%s'", node_name.c_str());
  }
  catch (const std::exception &e)
  {
    // Use default value if parameter is not provided
    node_name = "bota_driver_node";
    RCLCPP_WARN(temp_node->get_logger(),
                "Parameter 'node_name' was not provided. Using default: '%s'",
                node_name.c_str());
  }

  // Create the actual node with the node name
  return std::make_shared<BotaDriverNode>(node_name);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = create_bota_driver_node();
  if (!node)
  {
    rclcpp::shutdown();
    return 1;
  }
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
