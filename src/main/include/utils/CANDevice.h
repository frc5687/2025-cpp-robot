
#pragma once

#include <string_view>

struct CANDevice {
  int id;
  std::string_view bus;
};
