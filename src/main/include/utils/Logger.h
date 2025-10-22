
#pragma once

#include <networktables/BooleanArrayTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/FloatArrayTopic.h>
#include <networktables/FloatTopic.h>
#include <networktables/IntegerArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringArrayTopic.h>
#include <networktables/StringTopic.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>
#include <wpi/struct/Struct.h>

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

class Logger {
 public:
  static Logger& Instance() {
    static Logger inst;
    return inst;
  }

  template <typename T>
  void Log(const std::string& name, const T& value, int64_t ts = 0) {
    if constexpr (std::is_same_v<T, bool>) {
      auto& pub = GetOrCreate(name, m_boolPubs, [this, name] {
        return m_instance.GetBooleanTopic(name).Publish();
      });
      pub.Set(value, ts);

    } else if constexpr (std::is_same_v<T, int>) {
      auto& pub = GetOrCreate(name, m_intPubs, [this, name] {
        return m_instance.GetIntegerTopic(name).Publish();
      });
      pub.Set(static_cast<int64_t>(value), ts);

    } else if constexpr (std::is_same_v<T, float>) {
      auto& pub = GetOrCreate(name, m_floatPubs, [this, name] {
        return m_instance.GetFloatTopic(name).Publish();
      });
      pub.Set(value, ts);

    } else if constexpr (std::is_same_v<T, double>) {
      auto& pub = GetOrCreate(name, m_doublePubs, [this, name] {
        return m_instance.GetDoubleTopic(name).Publish();
      });
      pub.Set(value, ts);

    } else if constexpr (wpi::StructSerializable<T>) {
      auto& pub = GetOrCreate(name, m_structPubs<T>(), [this, name] {
        return m_instance.GetStructTopic<T>(name).Publish();
      });
      pub.Set(value, ts);

    } else {
      static_assert(!sizeof(T), "Type not supported");
    }
  }

  template <typename T>
  void Log(const std::string& name, const std::vector<T>& values,
           int64_t ts = 0) {
    if constexpr (std::is_same_v<T, bool>) {
      std::vector<int> tmp;
      tmp.reserve(values.size());
      for (bool b : values)
        tmp.push_back(b ? 1 : 0);
      auto& pub = GetOrCreate(name, m_boolArrPubs, [this, name] {
        return m_instance.GetBooleanArrayTopic(name).Publish();
      });
      pub.Set(tmp, ts);

    } else if constexpr (std::is_same_v<T, int>) {
      std::vector<int64_t> tmp(values.begin(), values.end());
      auto& pub = GetOrCreate(name, m_intArrPubs, [this, name] {
        return m_instance.GetIntegerArrayTopic(name).Publish();
      });
      pub.Set(tmp, ts);

    } else if constexpr (std::is_same_v<T, float>) {
      auto& pub = GetOrCreate(name, m_floatArrPubs, [this, name] {
        return m_instance.GetFloatArrayTopic(name).Publish();
      });
      pub.Set(values, ts);

    } else if constexpr (std::is_same_v<T, double>) {
      auto& pub = GetOrCreate(name, m_doubleArrPubs, [this, name] {
        return m_instance.GetDoubleArrayTopic(name).Publish();
      });
      pub.Set(values, ts);

    } else if constexpr (wpi::StructSerializable<T>) {
      auto& pub = GetOrCreate(name, m_structArrPubs<T>(), [this, name] {
        return m_instance.GetStructArrayTopic<T>(name).Publish();
      });
      pub.Set(values, ts);

    } else {
      static_assert(!sizeof(T), "Type not supported");
    }
  }

  template <typename T, size_t N>
  void Log(const std::string& name, const std::array<T, N>& arr,
           int64_t ts = 0) {
    std::vector<T> tmp(arr.begin(), arr.end());
    Log(name, tmp, ts);
  }

 private:
  Logger() : m_instance(nt::NetworkTableInstance::GetDefault()) {}

  template <class Map, class Factory>
  auto& GetOrCreate(const std::string& name, Map& map, Factory&& make) {
    std::scoped_lock lk(m_lock);
    if (auto it = map.find(name); it != map.end())
      return it->second;
    auto [it, _] = map.emplace(name, std::forward<Factory>(make)());
    return it->second;
  }

  nt::NetworkTableInstance m_instance;
  std::mutex m_lock;
  std::unordered_map<std::string, nt::BooleanPublisher> m_boolPubs;
  std::unordered_map<std::string, nt::IntegerPublisher> m_intPubs;
  std::unordered_map<std::string, nt::FloatPublisher> m_floatPubs;
  std::unordered_map<std::string, nt::DoublePublisher> m_doublePubs;
  std::unordered_map<std::string, nt::BooleanArrayPublisher> m_boolArrPubs;
  std::unordered_map<std::string, nt::IntegerArrayPublisher> m_intArrPubs;
  std::unordered_map<std::string, nt::FloatArrayPublisher> m_floatArrPubs;
  std::unordered_map<std::string, nt::DoubleArrayPublisher> m_doubleArrPubs;

  template <typename T>
  auto& m_structPubs() const {
    static std::unordered_map<std::string, nt::StructPublisher<T>> m;
    return m;
  }
  template <typename T>
  auto& m_structArrPubs() const {
    static std::unordered_map<std::string, nt::StructArrayPublisher<T>> m;
    return m;
  }
};
