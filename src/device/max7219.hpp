#pragma once

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <cstdint>
#include <stdexcept>

#include "bsp_gpio.hpp"
#include "bsp_spi.hpp"

template <size_t N>
class Max7219 {
 public:
  static constexpr uint8_t REG_NOOP = 0x00;
  static constexpr uint8_t REG_DIGIT0 = 0x01;
  static constexpr uint8_t REG_DIGIT7 = 0x08;
  static constexpr uint8_t REG_DECODE_MODE = 0x09;
  static constexpr uint8_t REG_INTENSITY = 0x0A;
  static constexpr uint8_t REG_SCAN_LIMIT = 0x0B;
  static constexpr uint8_t REG_SHUTDOWN = 0x0C;
  static constexpr uint8_t REG_DISPLAY_TEST = 0x0F;

  Max7219(SpiDevice& spi, Gpio* cs) : spi_(spi), cs_(cs) {
    if (!cs_) throw std::runtime_error("CS gpio is null");
    cs_->Write(1);
    for (auto& chip : framebuffer_) chip.fill(0);
  }

  void Initialize() {
    for (size_t i = 0; i < N; ++i) {
      WriteToChip(i, REG_SHUTDOWN, 0x00);  // 关闭
      usleep(5);
      WriteToChip(i, REG_DISPLAY_TEST, 0x00);  // 正常模式
      WriteToChip(i, REG_DECODE_MODE, 0x00);   // 点阵模式
      WriteToChip(i, REG_SCAN_LIMIT, 0x07);    // 显示8行
      WriteToChip(i, REG_INTENSITY, 0x08);     // 中等亮度
      WriteToChip(i, REG_SHUTDOWN, 0x01);      // 启动显示
    }
    Clear();
    Refresh();
  }

  void SetIntensity(uint8_t value) {
    if (value > 0x0F) value = 0x0F;
    WriteAll(REG_INTENSITY, value);
  }

  void Clear() {
    for (auto& chip : framebuffer_) chip.fill(0);
  }

  void DrawPixel(size_t chip_index, uint8_t row, uint8_t col, bool on) {
    if (chip_index >= N || row >= 8 || col >= 8) return;
    if (on) {
      framebuffer_[chip_index][7 - row] |= (1 << (col));
    } else {
      framebuffer_[chip_index][7 - row] &= ~(1 << (col));
    }
  }

  void DrawPixelMatrix2(uint8_t row, uint8_t col, bool on) {
    if (row >= 16 || col >= 32) {
      return;
    }

    static constexpr int chip_index_map[] = {0, 2, 1, 3};

    size_t chip_index = (row / 8) + (col / 8) * 2;
    chip_index = chip_index_map[chip_index % 4] + (chip_index - chip_index % 4);
    uint8_t local_row = row % 8;
    uint8_t local_col = col % 8;

    DrawPixel(chip_index, local_row, local_col, on);
  }

  void Refresh() {
    for (uint8_t row = 0; row < 8; ++row) {
      std::array<uint8_t, N> regs;
      std::array<uint8_t, N> data;
      regs.fill(REG_DIGIT0 + row);
      for (size_t i = 0; i < N; ++i) {
        data[i] = framebuffer_[i][row];
      }
      WriteCommandRaw(regs, data);
    }
  }

  void WriteToChip(size_t index, uint8_t addr, uint8_t data) {
    std::array<uint8_t, N> regs;
    std::array<uint8_t, N> data_all;
    regs.fill(REG_NOOP);
    data_all.fill(0x00);
    if (index >= N) return;
    regs[index] = addr;
    data_all[index] = data;
    WriteCommandRaw(regs, data_all);
  }

  void TestEachChip() {
    Clear();
    for (int i = 0; i < 16; ++i) {
      DrawPixelMatrix2(i, 0, true);
      Refresh();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    for (int i = 0; i < 16; ++i) {
      DrawPixelMatrix2(15, i, true);
      Refresh();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    for (int i = 0; i < 16; ++i) {
      DrawPixelMatrix2(i, 16, true);
      Refresh();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    for (int i = 16; i < 32; ++i) {
      DrawPixelMatrix2(15, i, true);
      Refresh();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    for (int i = 15; i > 0; i--) {
      DrawPixelMatrix2(i, 31, true);
      Refresh();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    for (int i = 31; i > 16; i--) {
      DrawPixelMatrix2(0, i, true);
      Refresh();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    for (int i = 15; i > 0; i--) {
      DrawPixelMatrix2(i, 15, true);
      Refresh();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    for (int i = 15; i > 0; i--) {
      DrawPixelMatrix2(0, i, true);
      Refresh();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

 private:
  SpiDevice& spi_;
  Gpio* cs_;
  std::array<std::array<uint8_t, 8>, N> framebuffer_;

  void WriteAll(uint8_t addr, uint8_t value) {
    std::array<uint8_t, N> data;
    data.fill(value);
    WriteCommand(addr, data);
  }

  void WriteCommand(uint8_t addr, const std::array<uint8_t, N>& data) {
    std::array<uint8_t, N> regs;
    regs.fill(addr);
    WriteCommandRaw(regs, data);
  }

  void WriteCommandRaw(const std::array<uint8_t, N>& regs,
                       const std::array<uint8_t, N>& data) {
    std::array<uint8_t, N * 2> tx_buf{};
    for (size_t i = 0; i < N; ++i) {
      const size_t hw_index = N - 1 - i;
      tx_buf[i * 2] = regs[hw_index];
      tx_buf[i * 2 + 1] = data[hw_index];
    }

    spi_ioc_transfer transfer{};
    transfer.tx_buf = reinterpret_cast<uint64_t>(tx_buf.data());
    transfer.len = tx_buf.size();
    transfer.speed_hz = 1000000;
    transfer.bits_per_word = 8;
    transfer.delay_usecs = 10;

    usleep(100);
    cs_->Write(0);
    usleep(100);

    if (ioctl(spi_.Fd(), SPI_IOC_MESSAGE(1), &transfer) < 0) {
      cs_->Write(1);
      throw std::runtime_error("SPI_IOC_MESSAGE failed");
    }

    usleep(100);
    cs_->Write(1);
    usleep(100);
  }
};
