
#pragma once
#include <map>
#include <vector>
#include <cstdint>

#include "bsp_gpio.hpp"

class SpiDevice {
 public:
  SpiDevice(const std::string& device, uint32_t speed, uint8_t mode) {}

  uint8_t ReadRegister(Gpio* cs, uint8_t reg) {
    return registers_[reg];
  }

  void WriteRegister(Gpio* cs, uint8_t reg, uint8_t value) {
    registers_[reg] = value;
  }

  void ReadRegisters(Gpio* cs, uint8_t reg, uint8_t* buffer, size_t length) {
    for (size_t i = 0; i < length; ++i) {
      buffer[i] = registers_[reg + i];
    }
  }

  int Fd() const { return 0; }

  uint32_t Speed() const { return 1000000; }

 private:
  std::map<uint8_t, uint8_t> registers_;
};
