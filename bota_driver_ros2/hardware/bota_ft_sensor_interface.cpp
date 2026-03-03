#include <optional>

#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <memory>

#include <rclcpp/logging.hpp>

#include "bota_driver_exposed.hpp"

namespace bota_driver
{

    class BotaFTSensor : public hardware_interface::SensorInterface
    {
    public:
        BotaFTSensor() = default;

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override
        {
            RCLCPP_INFO(rclcpp::get_logger("BotaFTSensor"), "Initializing sensor...");
            info_ = hardware_info;

            ///////////////////////////////////////////

            if (hardware_info.hardware_parameters.find("config_file") == hardware_info.hardware_parameters.end())
            {
                RCLCPP_ERROR(rclcpp::get_logger("BotaFTSensor"), "Missing 'config_file' parameter. Please provide an absolute path.");
                return hardware_interface::CallbackReturn::ERROR;
            }

            std::string config_file = hardware_info.hardware_parameters.at("config_file");

            /////////////////////////////////

            if (hardware_info.hardware_parameters.find("tare_on_activation") == hardware_info.hardware_parameters.end())
            {
                RCLCPP_WARN(rclcpp::get_logger("BotaFTSensor"), "Missing 'tare_on_activation' parameter. Defaulting to false.");
                tare_on_activation_ = false;
            }
            else
            {
                std::string tare_on_activation_str = hardware_info.hardware_parameters.at("tare_on_activation");
                if (tare_on_activation_str == "true")
                {
                    tare_on_activation_ = true;
                }
                else if (tare_on_activation_str == "false")
                {
                    tare_on_activation_ = false;
                }
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("BotaFTSensor"), "Invalid 'tare_on_activation' parameter value. Expected 'true' or 'false'.");
                    return hardware_interface::CallbackReturn::ERROR;
                }
            }

            /////////////////////////////////


            // Ensure the user provides an absolute path
            if (config_file.empty() || config_file[0] != '/')
            {
                RCLCPP_ERROR(rclcpp::get_logger("BotaFTSensor"), "The 'config_file' parameter must be an absolute path.");
                return hardware_interface::CallbackReturn::ERROR;
            }

            RCLCPP_INFO(rclcpp::get_logger("BotaFTSensor"), "Loading sensor config from: %s", config_file.c_str());

            driver_ = std::make_unique<bota::BotaDriverExposed>(config_file);

            // Set the hardware interface active flag to false
            hardware_interface_active_ = true;

            RCLCPP_INFO(rclcpp::get_logger("BotaFTSensor"), "Hardware interface initialized successfully");
            return hardware_interface::CallbackReturn::SUCCESS;
        }

        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            if (!driver_)
            {
                RCLCPP_ERROR(rclcpp::get_logger("BotaFTSensor"), "Driver is not initialized.");
                return hardware_interface::CallbackReturn::ERROR;
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("BotaFTSensor"), "Configuring hardware interface...");

                // Call the driver's onConfigure() function.
                if (!driver_->onConfigure())
                {
                    return hardware_interface::CallbackReturn::ERROR;
                }
            }
            RCLCPP_INFO(rclcpp::get_logger("BotaFTSensor"), "Hardware interface configured successfully");
            return hardware_interface::CallbackReturn::SUCCESS;
        }

        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            if (!driver_)
            {
                RCLCPP_ERROR(rclcpp::get_logger("BotaFTSensor"), "Driver is not initialized.");
                return hardware_interface::CallbackReturn::ERROR;
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("BotaFTSensor"), "Cleaning up hardware interface...");

                // Call the driver's onConfigure() function.
                if (!driver_->onCleanup())
                {
                    return hardware_interface::CallbackReturn::ERROR;
                }
            }
            RCLCPP_INFO(rclcpp::get_logger("BotaFTSensor"), "Hardware interface cleaned up successfully");
            return hardware_interface::CallbackReturn::SUCCESS;
        }

        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            if (!driver_)
            {
                RCLCPP_ERROR(rclcpp::get_logger("BotaFTSensor"), "Driver is not initialized.");
                return hardware_interface::CallbackReturn::ERROR;
            }

            ///////////////////////////////////////////////////////////////
            if (tare_on_activation_)
            {
                RCLCPP_INFO(rclcpp::get_logger("BotaFTSensor"), "Taring sensor...");
                if (!driver_->tare())
                {
                    RCLCPP_ERROR(rclcpp::get_logger("BotaFTSensor"), "Failed to tare the sensor");
                    return hardware_interface::CallbackReturn::ERROR;
                }
                RCLCPP_INFO(rclcpp::get_logger("BotaFTSensor"), "Sensor has been tared successfully");
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("BotaFTSensor"), "Taring sensor on activation is disabled.");
            }

            ///////////////////////////////////////////////////////////////

            RCLCPP_INFO(rclcpp::get_logger("BotaFTSensor"), "Activating hardware interface...");

            // Call the driver's onActivate() function.
            if (!driver_->onActivate())
            {
                return hardware_interface::CallbackReturn::ERROR;
            }

            // Set the hardware interface active flag to true
            hardware_interface_active_ = true;

            RCLCPP_INFO(rclcpp::get_logger("BotaFTSensor"), "Hardware interface activated successfully");
            return hardware_interface::CallbackReturn::SUCCESS;
        }

        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            if (!driver_)
            {
                RCLCPP_ERROR(rclcpp::get_logger("BotaFTSensor"), "Driver is not initialized.");
                return hardware_interface::CallbackReturn::ERROR;
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("BotaFTSensor"), "Deactivating sensor...");

                // Call the driver's onDeactivate() function.
                if (!driver_->onDeactivate())
                {
                    return hardware_interface::CallbackReturn::ERROR;
                }
            }

            // Set the hardware interface active flag to false
            hardware_interface_active_ = false;

            RCLCPP_INFO(rclcpp::get_logger("BotaFTSensor"), "Hardware interface deactivated successfully");
            return hardware_interface::CallbackReturn::SUCCESS;
        }

        hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            if (!driver_)
            {
                RCLCPP_ERROR(rclcpp::get_logger("BotaFTSensor"), "Driver is not initialized.");
                return hardware_interface::CallbackReturn::ERROR;
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("BotaFTSensor"), "Shutting down hardware interface...");

                // Call the driver's onDeactivate() function.
                if (!driver_->onShutdown())
                {
                    return hardware_interface::CallbackReturn::ERROR;
                }
            }

            // Set the hardware interface active flag to false
            hardware_interface_active_ = false;

            RCLCPP_INFO(rclcpp::get_logger("BotaFTSensor"), "Hardware interface shutdown successfully");
            return hardware_interface::CallbackReturn::SUCCESS;
        }

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override
        {
            std::vector<hardware_interface::StateInterface> state_interfaces;

            state_interfaces.emplace_back(get_sensor_name(), "hardware_interface_active", &hardware_interface_active_);

            state_interfaces.emplace_back(get_sensor_name(), "status", &status_);

            state_interfaces.emplace_back(get_sensor_name(), "status.throttled", &status_throttled_);
            state_interfaces.emplace_back(get_sensor_name(), "status.overrange", &status_overrange_);
            state_interfaces.emplace_back(get_sensor_name(), "status.invalid", &status_invalid_);
            state_interfaces.emplace_back(get_sensor_name(), "status.raw", &status_raw_);

            state_interfaces.emplace_back(get_sensor_name(), "force.x", &force_data_[0]);
            state_interfaces.emplace_back(get_sensor_name(), "force.y", &force_data_[1]);
            state_interfaces.emplace_back(get_sensor_name(), "force.z", &force_data_[2]);

            state_interfaces.emplace_back(get_sensor_name(), "torque.x", &torque_data_[0]);
            state_interfaces.emplace_back(get_sensor_name(), "torque.y", &torque_data_[1]);
            state_interfaces.emplace_back(get_sensor_name(), "torque.z", &torque_data_[2]);

            state_interfaces.emplace_back(get_sensor_name(), "lin_acc.x", &lin_acc_data_[0]);
            state_interfaces.emplace_back(get_sensor_name(), "lin_acc.y", &lin_acc_data_[1]);
            state_interfaces.emplace_back(get_sensor_name(), "lin_acc.z", &lin_acc_data_[2]);

            state_interfaces.emplace_back(get_sensor_name(), "ang_vel.x", &ang_vel_data_[0]);
            state_interfaces.emplace_back(get_sensor_name(), "ang_vel.y", &ang_vel_data_[1]);
            state_interfaces.emplace_back(get_sensor_name(), "ang_vel.z", &ang_vel_data_[2]);

            state_interfaces.emplace_back(get_sensor_name(), "temperature", &temperature_data_);
            state_interfaces.emplace_back(get_sensor_name(), "timestamp", &timestamp_data_);

            return state_interfaces;
        }

        // std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
        // {
        //     std::vector<hardware_interface::CommandInterface> command_interfaces;

        //     return command_interfaces;
        // }

        hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
        {
            // RCLCPP_DEBUG(rclcpp::get_logger("BotaFTSensor"), "Reading sensor data.");

            if (!driver_)
            {
                RCLCPP_ERROR(rclcpp::get_logger("BotaFTSensor"), "Driver is not initialized.");
                return hardware_interface::return_type::ERROR;
            }

            if (!hardware_interface_active_)
            {
                // RCLCPP_WARN(rclcpp::get_logger("BotaFTSensor"), "Hardware interface is not active.");

                // Fill the hardware interface with zeros
                status_throttled_ = 0.0;
                status_overrange_ = 0.0;
                status_invalid_ = 0.0;
                status_raw_ = 0.0;

                for (size_t i = 0; i < 3; ++i)
                {
                    force_data_[i] = 0.0;
                    torque_data_[i] = 0.0;
                    lin_acc_data_[i] = 0.0;
                    ang_vel_data_[i] = 0.0;
                }

                temperature_data_ = 0.0;
                timestamp_data_ = 0.0;

                return hardware_interface::return_type::OK;
            }

            // If active, read the sensor data

            try
            {
                bota_frame_ = driver_->readFrame();
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("BotaFTSensor"), "Error reading sensor data: %s", e.what());
                return hardware_interface::return_type::ERROR;
            }

            // Copy the data to the double variables that are exposed through the interfaces
            status_throttled_ = bota_frame_.status.bits.throttled ? 1.0 : 0.0;
            status_overrange_ = bota_frame_.status.bits.overrange ? 1.0 : 0.0;
            status_invalid_ = bota_frame_.status.bits.invalid ? 1.0 : 0.0;
            status_raw_ = bota_frame_.status.bits.raw ? 1.0 : 0.0;

            status_ = bota_frame_.status.bits.throttled &&
                              bota_frame_.status.bits.overrange &&
                              bota_frame_.status.bits.invalid &&
                              bota_frame_.status.bits.raw
                          ? 1.0
                          : 0.0;

            for (size_t i = 0; i < 3; ++i)
            {
                force_data_[i] = bota_frame_.force[i];
                torque_data_[i] = bota_frame_.torque[i];
                lin_acc_data_[i] = bota_frame_.acceleration[i];
                ang_vel_data_[i] = bota_frame_.angular_rate[i];
            }

            temperature_data_ = bota_frame_.temperature;
            timestamp_data_ = static_cast<double>(bota_frame_.timestamp);

            return hardware_interface::return_type::OK;
        }

        // hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
        // {

        //     return hardware_interface::return_type::OK;
        // }

    private:
        std::string get_sensor_name() const
        {
            if (info_.name.empty())
            {
                RCLCPP_WARN(rclcpp::get_logger("BotaFTSensor"),
                            "Sensor name not found; defaulting to 'bota_ft_sensor'");
                return std::string("bota_ft_sensor");
            }
            return info_.name;
        }

        // Variable to hold the hardware interface active state
        double hardware_interface_active_{0.0};

        // Variables to hold the frame values
        double status_{0.0}; // summary of all the status bits
        double status_throttled_{0.0};
        double status_overrange_{0.0};
        double status_invalid_{0.0};
        double status_raw_{0.0};
        std::array<double, 3> force_data_ = {0.0, 0.0, 0.0};
        std::array<double, 3> torque_data_ = {0.0, 0.0, 0.0};
        std::array<double, 3> lin_acc_data_ = {0.0, 0.0, 0.0};
        std::array<double, 3> ang_vel_data_ = {0.0, 0.0, 0.0};
        double temperature_data_ = 0.0;
        double timestamp_data_ = 0.0;

        std::unique_ptr<bota::BotaDriverExposed> driver_;
        bota::BotaFrame bota_frame_{bota::DataStatus{}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, 0, 0.0f, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};

        bool tare_on_activation_{false};
    };

} // namespace bota_driver

// Register as a plugin
PLUGINLIB_EXPORT_CLASS(bota_driver::BotaFTSensor, hardware_interface::SensorInterface)
