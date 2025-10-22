
// ElevatorState.h
#pragma once

#include <units/acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

#include <optional>

class ElevatorState {
 public:
  struct Snapshot {
    units::second_t stamp{0_s};

    units::meter_t position{0_m};
    units::meters_per_second_t velocity{0_mps};
    units::meters_per_second_squared_t accel{0_mps_sq};

    std::optional<units::meter_t> goal{};
  };

  void Update(const Snapshot& s) { m_state = s; }
  Snapshot GetSnapshot() const { return m_state; }
  void SetGoal(units::meter_t g) { m_state.goal = g; }

 private:
  Snapshot m_state{};
};
