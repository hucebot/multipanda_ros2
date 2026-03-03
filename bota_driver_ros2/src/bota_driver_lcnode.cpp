#include "bota_driver/bota_driver_lcnode.hpp"
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "bota_driver/msg/bota_frame.hpp"

#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

BotaDriverLCNode::BotaDriverLCNode()
    : LifecycleNode("bota_driver_lcnode")
{
    // 1) Declare without defaults → rclcpp will throw if missing.
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

    try
    {
        // 1) Try to declare it as a double
        output_rate_ = declare_parameter<float>("output_rate");
        RCLCPP_INFO(get_logger(), "Using output rate: %.2f Hz", output_rate_);
    }
    catch (const rclcpp::exceptions::InvalidParameterTypeException &e)
    {
        // 2) They passed an integer → redeclare as int to pick it up
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
        // 3) Any other problem (missing entirely, bad name, etc) → fatal
        RCLCPP_FATAL(get_logger(),
                     "Required parameter 'output_rate' was not provided or is invalid: %s",
                     e.what());
        rclcpp::shutdown();
        return;
    }

    try
    {
        bota_ft_sensor_link_name_ = declare_parameter<std::string>("bota_ft_sensor_link_name");
        RCLCPP_INFO(get_logger(), "Using sensor link name: '%s'", bota_ft_sensor_link_name_.c_str());
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(get_logger(),
                     "Required parameter 'bota_ft_sensor_link_name' was not provided. "
                     "Please set it in your launch file or via --ros-args -p bota_ft_sensor_link_name:=<frame_id>");
        rclcpp::shutdown();
        return;
    }

    // 2) Initialize driver with the user-provided config file
    driver_ = std::make_unique<bota::BotaDriverExposed>(config_file_);

    // 3) Now set up your publishers, configure & activate, etc.
    publisher_bota_wrench_ =
        create_publisher<geometry_msgs::msg::WrenchStamped>("bota_wrench", 10);
    publisher_bota_imu_ =
        create_publisher<sensor_msgs::msg::Imu>("bota_imu", 10);
    publisher_bota_frame_ =
        create_publisher<bota_driver::msg::BotaFrame>("bota_frame", 10);

    // Add tare service
    tare_service_ = create_service<std_srvs::srv::Trigger>(
        "/bota_driver_lcnode/tare", std::bind(&BotaDriverLCNode::tare_callback, this,
                                              std::placeholders::_1, std::placeholders::_2));
}

BotaDriverLCNode::~BotaDriverLCNode()
{
    RCLCPP_INFO(get_logger(), "bota_driver_lcnode shutting down");
    stop_async_reading();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BotaDriverLCNode::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Configuring BotaDriverLCNode");

    try
    {
        driver_->onConfigure();

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(get_logger(), "Error in configure: %s", e.what());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BotaDriverLCNode::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning up BotaDriverLCNode");

    if (driver_)
    {
        driver_->onCleanup();
    }

    timer_.reset();
    stop_async_reading();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BotaDriverLCNode::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Activating BotaDriverLCNode");

    // Activate all publishers
    publisher_bota_wrench_->on_activate();
    publisher_bota_imu_->on_activate();
    publisher_bota_frame_->on_activate();

    // Activate driver
    driver_->onActivate();

    // Start sync or async reading
    if (output_rate_ < 0.0)
    {
        RCLCPP_ERROR(get_logger(),
                     "output_rate must be >= 0 (0 for async, >0 for sync)");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    else if (output_rate_ == 0.0)
    {
        RCLCPP_INFO(get_logger(),
                    "Async reading: topic rate set by sensor update rate");
        start_async_reading();
    }
    else
    {
        RCLCPP_INFO(get_logger(),
                    "Sync reading: polling data from the sensor and publishing at %.2f Hz", output_rate_);

        auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / output_rate_));

        timer_ = create_wall_timer(
            period, std::bind(&BotaDriverLCNode::read_sync_and_publish, this));
    }

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BotaDriverLCNode::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Deactivating BotaDriverLCNode");

    // Stop reading
    timer_.reset();
    stop_async_reading();

    // Deactivate driver
    driver_->onDeactivate();

    // Deactivate all publishers
    publisher_bota_wrench_->on_deactivate();
    publisher_bota_imu_->on_deactivate();
    publisher_bota_frame_->on_deactivate();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BotaDriverLCNode::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Shutting down BotaDriverLCNode");

    // Stop reading
    timer_.reset();
    stop_async_reading();

    // Shutdown driver
    if (driver_)
    {
        driver_->onDeactivate();
        driver_->onCleanup();
        driver_->onShutdown();
    }

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BotaDriverLCNode::on_error(const rclcpp_lifecycle::State &)
{
    RCLCPP_ERROR(get_logger(), "Error in BotaDriverLCNode");

    if (driver_)
    {
        driver_->onError();
    }

    // Stop reading
    timer_.reset();
    stop_async_reading();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void BotaDriverLCNode::publish_frame_data(const bota::BotaFrame &frame)
{
    // Only publish if the node is active
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        return;
    }

    // Get the current time so the topics can be stamped with the same time
    rclcpp::Time stamp = now();

    // Wrench
    geometry_msgs::msg::WrenchStamped wrench_msg;
    wrench_msg.header.stamp = stamp;
    wrench_msg.header.frame_id = bota_ft_sensor_link_name_ + "_wrench";
    wrench_msg.wrench.force.x = frame.force[0];
    wrench_msg.wrench.force.y = frame.force[1];
    wrench_msg.wrench.force.z = frame.force[2];
    wrench_msg.wrench.torque.x = frame.torque[0];
    wrench_msg.wrench.torque.y = frame.torque[1];
    wrench_msg.wrench.torque.z = frame.torque[2];
    publisher_bota_wrench_->publish(wrench_msg);

    // Imu
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = stamp;
    imu_msg.header.frame_id = bota_ft_sensor_link_name_ + "_imu";
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

void BotaDriverLCNode::read_sync_and_publish()
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

void BotaDriverLCNode::start_async_reading()
{
    async_thread_running_ = true;
    async_thread_ = std::thread(&BotaDriverLCNode::async_reading_loop, this);
}

void BotaDriverLCNode::async_reading_loop()
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

void BotaDriverLCNode::stop_async_reading()
{
    async_thread_running_ = false;
    if (async_thread_.joinable())
    {
        async_thread_.join();
    }
}

void BotaDriverLCNode::tare_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request; // Mark as unused

    try
    {
        // Call the tare method
        if (driver_->tare())
        {
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
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(get_logger(), "Failed to tare sensor: %s", e.what());
        response->success = false;
        response->message = "Failed to tare sensor: " + std::string(e.what());
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<BotaDriverLCNode>();
    exec.add_node(node->get_node_base_interface());
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
