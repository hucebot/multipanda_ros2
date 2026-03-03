/**
 * @file bota_driver.hpp
 * @brief Public header file for the BotaDriver class.
 * @brief More information about the BotaDriver class and its functionality can be found in the
 * documentation. https://code.botasys.com/en/latest/layer1/driver.html
 *
 * @copyright
 * Copyright (c) 2024 Botasys AG
 * SPDX-License-Identifier: BSL3
 * This driver is part of the BotaDriver project and is licensed under the BSL3 License.
 * See the LICENSE file accompanying this library for more information.
 */

#ifndef BOTA_DRIVER_HPP
#define BOTA_DRIVER_HPP

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

// Platform-specific DLL export/import macros
#ifdef _WIN32
#if defined(BOTA_DRIVER_EXPORTS) || defined(BOTA_DRIVER_EXPOSED_EXPORTS)
#define BOTA_DRIVER_API __declspec(dllexport)
#else
#define BOTA_DRIVER_API __declspec(dllimport)
#endif
#else
#define BOTA_DRIVER_API
#endif

// Platform-specific packed structure macros
#ifdef _MSC_VER
#define PACKED_BEGIN __pragma(pack(push, 1))
#define PACKED_END __pragma(pack(pop))
#define PACKED_STRUCT
#elif defined(__GNUC__)
#define PACKED_BEGIN
#define PACKED_END
#define PACKED_STRUCT __attribute__((packed))
#else
#define PACKED_BEGIN
#define PACKED_END
#define PACKED_STRUCT
#endif
namespace bota {

/**
 * @enum DriverState
 * @brief Represents the lifecycle primary states and transition states of the driver.
 */
enum class DriverState {
  INITIAL,          ///< Initial state after construction
  UNCONFIGURED,     ///< Driver is in unconfigured state
  INACTIVE,         ///< Driver is in inactive state
  ACTIVE,           ///< Driver is in active state
  FINALIZED,        ///< Driver has been finalized
  TERMINAL,         ///< Terminal state, unrecoverable
  CONFIGURING,      ///< Driver is configuring
  CLEANING_UP,      ///< Driver is cleaning up
  SHUTTING_DOWN,    ///< Driver is shutting down
  ACTIVATING,       ///< Driver is activating
  DEACTIVATING,     ///< Driver is deactivating
  ERROR_PROCESSING  ///< Driver is handling an error
};

/**
 * @union DataStatus
 * @brief Status flags for sensor data.
 */
PACKED_BEGIN
union DataStatus {
  struct PACKED_STRUCT {
    uint16_t throttled : 1;  ///< Data is throttled
    uint16_t overrange : 1;  ///< Data is overrange
    uint16_t invalid : 1;    ///< Data is invalid
    uint16_t raw : 1;        ///< Data is raw
    uint16_t : 12;           ///< Reserved
  } bits;
  uint16_t val;      ///< Status as uint16_t
  uint8_t bytes[1];  ///< Status as bytes
};
PACKED_END

/**
 * @struct BotaFrame
 * @brief Represents a single sensor data frame.
 */
struct BotaFrame {
  DataStatus status;                  ///< Status flags
  std::array<float, 3> force;         ///< Force vector
  std::array<float, 3> torque;        ///< Torque vector
  uint32_t timestamp;                 ///< Timestamp of the frame
  float temperature;                  ///< Temperature reading
  std::array<float, 3> acceleration;  ///< Acceleration vector
  std::array<float, 3> angular_rate;  ///< Angular rate vector

  /**
   * @brief Default constructor - initializes all members to zero.
   */
  BotaFrame()
      : status{},
        force{0.0f, 0.0f, 0.0f},
        torque{0.0f, 0.0f, 0.0f},
        timestamp(0),
        temperature(0.0f),
        acceleration{0.0f, 0.0f, 0.0f},
        angular_rate{0.0f, 0.0f, 0.0f} {}

  /**
   * @brief Constructor for BotaFrame.
   * @param status_data Status flags
   * @param force_data Force vector
   * @param torque_data Torque vector
   * @param timestamp_data Timestamp
   * @param temperature_data Temperature
   * @param acceleration_data Acceleration vector
   * @param angular_rate_data Angular rate vector
   */
  BotaFrame(DataStatus status_data, std::array<float, 3> force_data,
            std::array<float, 3> torque_data, uint32_t timestamp_data, float temperature_data,
            std::array<float, 3> acceleration_data, std::array<float, 3> angular_rate_data)
      : status(status_data),
        force(force_data),
        torque(torque_data),
        timestamp(timestamp_data),
        temperature(temperature_data),
        acceleration(acceleration_data),
        angular_rate(angular_rate_data) {}
};

class BotaDriverExposed;  // to allow friendship

/**
 * @class BotaDriver
 * @brief Public API for managing the lifecycle of a sensor driver.
 */
class BOTA_DRIVER_API BotaDriver {
 public:
  /**
   * @brief Default constructor.
   */
  BotaDriver();

  /**
   * @brief Constructor with configuration path.
   * @param config_path Path to JSON configuration file
   */
  explicit BotaDriver(const std::string &config_path);

  /**
   * @brief Destructor.
   */
  ~BotaDriver();

  /**
   * @brief Get driver version generation.
   * @return Generation number
   */
  [[nodiscard]] uint16_t getDriverVersionGeneration() const;

  /**
   * @brief Get driver version major.
   * @return Major version number
   */
  [[nodiscard]] uint16_t getDriverVersionMajor() const;

  /**
   * @brief Get driver version minor.
   * @return Minor version number
   */
  [[nodiscard]] uint16_t getDriverVersionMinor() const;

  /**
   * @brief Get driver version as string.
   * @return Version string
   */
  [[nodiscard]] std::string getDriverVersionString() const;

  /**
   * @brief Trigger the transition from UNCONFIGURED towards INACTIVE.
   * @return True if successful
   */
  [[nodiscard]] bool configure();

  /**
   * @brief Trigger the transition from INACTIVE towards ACTIVE.
   * @return True if successful
   */
  [[nodiscard]] bool activate();

  /**
   * @brief Trigger the transition from ACTIVE towards INACTIVE.
   * @return True if successful
   */
  [[nodiscard]] bool deactivate();

  /**
   * @brief Trigger the transition from INACTIVE towards UNCONFIGURED.
   * @return True if successful
   */
  [[nodiscard]] bool cleanup();

  /**
   * @brief Shutdown the driver.
   * @return True if successful
   */
  [[nodiscard]] bool shutdown();

  /**
   * @brief Get the current driver state.
   * @return Current state
   */
  [[nodiscard]] DriverState getDriverState() const;

  /**
   * @brief Get the expected timestep for data acquisition.
   * @return Expected timestep in microseconds
   */
  [[nodiscard]] std::chrono::microseconds getExpectedTimestep() const;

  /**
   * @brief Perform tare operation (zeroing).
   * @return True if successful
   * @note Can only be called in INACTIVE state
   */
  [[nodiscard]] bool tare() const;

  /**
   * @brief Read a single data frame from the buffer (non-blocking).
   * @return Sensor data frame
   * @note Can only be called in ACTIVE state
   */
  [[nodiscard]] BotaFrame readFrame() const;

  /**
   * @brief Read a single data frame from the stream (blocking until new frame is sampled).
   * @return Sensor data frame
   * @note Can only be called in ACTIVE state
   * @note Only available for streaming communication interfaces
   */
  [[nodiscard]] BotaFrame readFrameBlocking() const;

 private:
  // Implementation details - forward declaration
  class Impl;                   ///< Implementation class
  std::unique_ptr<Impl> impl_;  ///< Pointer to implementation
  friend class BotaDriverExposed;
};

}  // namespace bota

#endif  // BOTA_DRIVER_HPP
