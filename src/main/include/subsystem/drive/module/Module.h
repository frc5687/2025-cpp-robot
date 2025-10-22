
// Module.h
#pragma once

#include <frc/Timer.h>
#include <frc/filter/LinearFilter.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>

#include <memory>
#include <string>

#include "ModuleIO.h"
#include "units/velocity.h"

class Module {
 public:
  explicit Module(std::unique_ptr<ModuleIO> io);

  // Core functionality
  void Periodic();
  void SetDesiredState(const frc::SwerveModuleState& state);
  void SetDesiredState(const frc::SwerveModuleState& state, bool optimize);
  void Stop();

  frc::SwerveModuleState GetState() const;
  frc::SwerveModulePosition GetPosition() const;
  frc::SwerveModuleState GetOptimizedState() const { return m_optimizedState; }
  const ModuleIOInputs& GetInputs() const { return m_inputs; }

  const std::string& GetName() const { return m_name; }
  ModulePosition GetModulePosition() const { return m_position; }
  units::millisecond_t GetLastUpdateTime() const { return m_lastUpdateTime; }

  void ResetDrivePosition() const;
  void SetBrakeMode(bool brake);
  units::ampere_t GetCurrentDraw() const;
  units::newton_meter_t GetDriveTorque() const { return m_inputs.driveTorque; }
  void ConfigureClosedLoop();
  bool IsConnected() const;
  void SetIsBatchedSignals(const bool& isBatched) {
    m_isSignalsBatched = isBatched;
  }
  bool IsBatched() const { return m_isSignalsBatched; }
  ModuleIO& GetModuleIO() { return *m_io; }
  const ModuleIO& GetModuleIO() const { return *m_io; }

 private:
  std::unique_ptr<ModuleIO> m_io;

  ModulePosition m_position;
  std::string m_name;

  ModuleIOInputs m_inputs;
  frc::SwerveModuleState m_desiredState;
  frc::SwerveModuleState m_optimizedState;

  // We are using this in a const fn, so need to set to mutable.
  mutable frc::LinearFilter<double> m_velocityFilter{
      frc::LinearFilter<double>::MovingAverage(3)};
  mutable frc::LinearFilter<double> m_currentFilter{
      frc::LinearFilter<double>::MovingAverage(5)};

  units::millisecond_t m_lastUpdateTime{0};
  units::second_t m_lastLogTime{0};
  bool m_isSignalsBatched = false;

  static constexpr units::meters_per_second_t kOptimizationThreshold = 0.01_mps;

  void LogState();
  bool ShouldOptimize(const frc::SwerveModuleState& desired) const;
};
