#ifndef BOTA_DRIVER_EXPOSED_HPP
#define BOTA_DRIVER_EXPOSED_HPP

#include <chrono>
#include <string>

#include "bota_driver.hpp"

// Platform-specific DLL export/import macros for exposed library
#ifdef _WIN32
#ifdef BOTA_DRIVER_EXPOSED_EXPORTS
#define BOTA_DRIVER_EXPOSED_API __declspec(dllexport)
#else
#define BOTA_DRIVER_EXPOSED_API __declspec(dllimport)
#endif
#else
#define BOTA_DRIVER_EXPOSED_API
#endif

namespace bota {

class BOTA_DRIVER_EXPOSED_API BotaDriverExposed : public BotaDriver {
 public:
  BotaDriverExposed() = default;  // Allows construction without a config file
  explicit BotaDriverExposed(const std::string &config_path);
  ~BotaDriverExposed();

  bool onConfigure();
  bool onCleanup();
  bool onActivate();
  bool onDeactivate();
  bool onShutdown();
  bool onError();

  [[nodiscard]] DriverState getDriverState() const;
  [[nodiscard]] BotaFrame readFrame() const;
  [[nodiscard]] BotaFrame readFrameBlocking() const;
  [[nodiscard]] std::chrono::microseconds getExpectedTimestep() const;

  bool tare();
};

}  // namespace bota

#endif  // BOTA_DRIVER_EXPOSED_HPP