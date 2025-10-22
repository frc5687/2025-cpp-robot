
#pragma once

#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <memory>
#include <string>

class TunableDouble {
 public:
  TunableDouble(const std::string& table, const std::string& topic,
                double defaultValue = 0.0);

  // Full NT path (e.g., "/SmartDashboard/kP" or "/datatable/kP")
  explicit TunableDouble(const std::string& fullTopicPath,
                         double defaultValue = 0.0);

  bool HasChanged();
  double Get() const noexcept { return m_cached; }

  int64_t LastChangeUsec() const noexcept { return m_lastChange; }

 private:
  void Init(nt::DoubleTopic topic, double defaultValue);

  std::shared_ptr<nt::NetworkTable>
      m_table;  // optional (unused when full path ctor is used)
  nt::DoubleTopic m_topic;
  nt::DoubleEntry m_entry;

  double m_defaultValue = 0.0;
  double m_cached = 0.0;
  int64_t m_lastChange = 0;
};
