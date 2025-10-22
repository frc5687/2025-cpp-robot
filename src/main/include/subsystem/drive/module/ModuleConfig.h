
#pragma once

#include <units/angle.h>

#include "ModulePosition.h"

struct ModuleConfig {
  ModulePosition modulePosition;
  units::turn_t encoderOffset{0.0_tr};

  explicit ModuleConfig(ModulePosition position, units::turn_t offset)
      : modulePosition(position), encoderOffset(offset) {}
};
