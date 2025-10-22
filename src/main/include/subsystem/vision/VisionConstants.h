
#pragma once

#include <frc/geometry/Transform3d.h>
#include <units/angle.h>
#include <units/length.h>

#include <string>
#include <unordered_map>

namespace Constants::Vision {
using units::meter_t;
using units::radian_t;

inline constexpr frc::Transform3d kRobotToNWCam{
    frc::Translation3d{meter_t{0.0}, meter_t{0.0}, meter_t{0.0}},
    frc::Rotation3d{0_rad, 0_rad, 0_rad}  // yaw/pitch/roll (radians)
};

inline constexpr frc::Transform3d kRobotToNECam{
    frc::Translation3d{meter_t{0.0}, meter_t{0.0}, meter_t{0.0}},
    frc::Rotation3d{0_rad, 0_rad, 0_rad}};

inline constexpr frc::Transform3d kRobotToSouthCam{
    frc::Translation3d{meter_t{0.0}, meter_t{0.0}, meter_t{0.0}},
    frc::Rotation3d{0_rad, 0_rad, 0_rad}};

inline const std::unordered_map<std::string, frc::Transform3d> kTransformMap = {
    {"limelightleft", kRobotToNWCam},
    {"limelight-center", kRobotToNECam},
    {"South_Camera", kRobotToSouthCam},
};
}  // namespace Constants::Vision
