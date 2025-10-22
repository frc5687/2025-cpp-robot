
#include "utils/TunableDouble.h"

#include <string>

void TunableDouble::Init(nt::DoubleTopic topic, double defaultValue) {
  m_topic = topic;
  m_defaultValue = defaultValue;

  m_entry = m_topic.GetEntry(m_defaultValue);

  m_entry.SetDefault(m_defaultValue);
  m_cached = m_entry.Get(m_defaultValue);
  m_lastChange = m_entry.GetLastChange();
}

TunableDouble::TunableDouble(const std::string& table, const std::string& topic,
                             double defaultValue) {
  m_table = nt::NetworkTableInstance::GetDefault().GetTable(table);
  Init(m_table->GetDoubleTopic(topic), defaultValue);
}

TunableDouble::TunableDouble(const std::string& fullTopicPath,
                             double defaultValue) {
  auto inst = nt::NetworkTableInstance::GetDefault();
  Init(inst.GetDoubleTopic(fullTopicPath), defaultValue);
}

bool TunableDouble::HasChanged() {
  auto updates = m_entry.ReadQueue();
  if (updates.empty())
    return false;

  const auto& last = updates.back();
  m_cached = last.value;
  m_lastChange = last.time;
  return true;
}
