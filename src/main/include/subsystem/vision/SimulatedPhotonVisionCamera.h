
#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform3d.h>
#include <photon/simulation/PhotonCameraSim.h>
#include <photon/simulation/SimCameraProperties.h>
#include <photon/simulation/VisionSystemSim.h>
#include <units/angle.h>
#include <units/frequency.h>
#include <units/time.h>

#include <memory>
#include <string>
#include <utility>

#include "PhotonVisionCamera.h"

class SimulatedPhotonVisionCamera : public PhotonVisionCamera {
 public:
  SimulatedPhotonVisionCamera(
      const std::string& name, const frc::Transform3d& robotToCamera,
      std::shared_ptr<photon::VisionSystemSim> visionSim)
      : PhotonVisionCamera(name, robotToCamera),
        m_cameraSim(&m_camera, MakeProps()),
        m_visionSim(std::move(visionSim)) {
    if (m_visionSim) {
      m_visionSim->AddCamera(&m_cameraSim, m_robotToCamera);
    }
    // This is breaking in SIM dont know why?
    m_camera.SetVersionCheckEnabled(false);
    m_cameraSim.EnableDrawWireframe(false);
    m_cameraSim.EnabledProcessedStream(false);
    m_cameraSim.EnableRawStream(false);
    m_cameraSim.SetMaxSightRange(3.0_m);
  }

 private:
  static photon::SimCameraProperties MakeProps() {
    photon::SimCameraProperties props;
    props.SetCalibration(1280, 720, frc::Rotation2d{88_deg});  // diagonal FOV
    props.SetCalibError(1.35, 0.10);
    props.SetFPS(60_Hz);
    props.SetAvgLatency(20_ms);
    props.SetLatencyStdDev(5_ms);
    return props;
  }

  photon::PhotonCameraSim m_cameraSim;
  std::shared_ptr<photon::VisionSystemSim> m_visionSim;
};
