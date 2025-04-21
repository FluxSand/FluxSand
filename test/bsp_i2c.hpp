
#pragma once
#include <map>
#include <vector>
#include <stdexcept>

class I2cDevice {
 public:
  I2cDevice(const std::string& device, uint8_t addr) : addr_(addr) {}

  uint8_t ReadRegister(uint8_t reg) {
    return registers_[reg];
  }

  void WriteRegister(uint8_t reg, uint8_t value) {
    registers_[reg] = value;
  }

  void ReadRegisters(uint8_t reg, uint8_t* buffer, size_t length) {
    for (size_t i = 0; i < length; ++i) {
      buffer[i] = registers_[reg + i];
    }
  }

  void WriteRaw(const uint8_t* data, size_t length) {
    for (size_t i = 0; i + 1 < length; i += 2) {
      registers_[data[i]] = data[i + 1];
    }
  }

 private:
  uint8_t addr_;
  std::map<uint8_t, uint8_t> registers_;
};
