
#pragma once

#include <string>

enum class ModulePosition { FrontLeft, FrontRight, BackLeft, BackRight };

inline constexpr std::string ModulePositionToString(ModulePosition pos) {
  switch (pos) {
    case ModulePosition::FrontLeft:
      return "FL";
    case ModulePosition::FrontRight:
      return "FR";
    case ModulePosition::BackLeft:
      return "BL";
    case ModulePosition::BackRight:
      return "BR";
    default:
      return "Unknown";
  }
}
