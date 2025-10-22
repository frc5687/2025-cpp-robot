
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <units/time.h>

#include <memory>
#include <string>

#include "utils/Logger.h"

/**
 * @brief Base class for subsystems with automatic logging and NetworkTables
 * integration.
 */
class LoggedSubsystem : public frc2::SubsystemBase {
 public:
  /**
   * @brief Construct a logged subsystem.
   * @param name Subsystem name used for logging and NetworkTables
   */
  explicit LoggedSubsystem(const std::string& name);

  /**
   * @brief Main periodic function that handles logging automatically.
   *
   * Calls UpdateInputs() and LogTelemetry(). Subclasses override the virtual
   * methods instead of this one.
   */
  void Periodic() final;

 protected:
  /**
   * @brief Update inputs from hardware.
   *
   * Called every robot loop (20ms). Use this to read sensor values,
   * motor positions, etc.
   */
  virtual void UpdateInputs() {}

  /**
   * @brief Log telemetry data.
   * Use this to send data to the logger and NetworkTables.
   */
  virtual void LogTelemetry() {}

  /**
   * @brief Get the NetworkTables table for this subsystem.
   */
  std::shared_ptr<nt::NetworkTable> GetTable() const { return m_table; }

  /**
   * @brief Log a value with the subsystem prefix.
   *
   * @param key The key to log (will be prefixed with subsystem name)
   * @param value The value to log
   */
  template <typename T>
  void Log(const std::string& key, const T& value) {
    Logger::Instance().Log(m_name + "/" + key, value);
  }

  std::string m_name;

 private:
  std::shared_ptr<nt::NetworkTable> m_table;
  units::second_t m_lastLogTime{0};

  units::second_t m_totalUpdateTime{0};
  size_t m_updateCount{0};
};
