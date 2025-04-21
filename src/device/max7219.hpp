#pragma once

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <thread>

#include "bsp_gpio.hpp"
#include "bsp_spi.hpp"

/* MAX7219 LED matrix driver controller template class
 * N: Number of cascaded MAX7219 chips */
template <size_t N>
class Max7219 {
 public:
  /* Register address definitions */
  static constexpr uint8_t REG_NOOP = 0x00;
  static constexpr uint8_t REG_DIGIT0 = 0x01;
  static constexpr uint8_t REG_DIGIT7 = 0x08;
  static constexpr uint8_t REG_DECODE_MODE = 0x09;
  static constexpr uint8_t REG_INTENSITY = 0x0A;
  static constexpr uint8_t REG_SCAN_LIMIT = 0x0B;
  static constexpr uint8_t REG_SHUTDOWN = 0x0C;
  static constexpr uint8_t REG_DISPLAY_TEST = 0x0F;

  /* Constructor: Initialize SPI and CS (Chip Select) pin */
  Max7219(SpiDevice& spi, Gpio* cs) : spi_(spi), cs_(cs) {
    if (!cs_) {
      std::perror("CS GPIO is not initialized");
    }
    cs_->Write(1); /* CS active low, initialize to high */
    for (auto& chip : framebuffer_) {
      chip.fill(0); /* Clear frame buffer */
    }

    thread_ = std::thread(&Max7219::RefreshThread, this);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    TestEachChip();
  }

  void RefreshThread() {
    Initialize();

    while (true) {
      Refresh();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  /* Initialize all cascaded chips */
  void Initialize() {
    for (size_t i = 0; i < N; ++i) {
      WriteToChip(i, REG_SHUTDOWN, 0x00);     /* Enter shutdown mode */
      usleep(5);                              /* Short delay */
      WriteToChip(i, REG_DISPLAY_TEST, 0x00); /* Normal operation */
      WriteToChip(i, REG_DECODE_MODE, 0x00);  /* Matrix mode (no decoding) */
      WriteToChip(i, REG_SCAN_LIMIT, 0x07);   /* Scan all 8 rows */
      WriteToChip(i, REG_INTENSITY, 0x03);    /* Medium brightness */
      WriteToChip(i, REG_SHUTDOWN, 0x01);     /* Normal operation */
    }
    Clear();
    Refresh();
  }

  /* Set global brightness (0-15) */
  void SetIntensity(uint8_t value) {
    mutex_.lock();
    if (value > 0x0F) {
      value = 0x0F;
    }
    WriteAll(REG_INTENSITY, value);
    mutex_.unlock();
  }

  /* Clear frame buffer */
  void Clear() {
    for (auto& chip : framebuffer_) {
      chip.fill(0);
    }
  }

  /* Set individual pixel state
   * chip_index: Chip index (0-based)
   * row: Vertical position (0-7)
   * col: Horizontal position (0-7) */
  void DrawPixel(size_t chip_index, uint8_t row, uint8_t col, bool on) {
    if (chip_index >= N || row >= 8 || col >= 8) {
      return;
    }
    if (on) {
      framebuffer_[chip_index][7 - row] |= (1 << col);
    } else {
      framebuffer_[chip_index][7 - row] &= ~(1 << col);
    }
  }

  /* 16x32 composite matrix drawing function
   * Handles coordinate mapping for 4x2 matrix layout */
  void DrawPixelMatrix2(uint8_t row, uint8_t col, bool on) {
    if (row >= 16 || col >= 32) {
      return;
    }

    /* Serpentine layout chip index mapping */
    static constexpr int CHIP_INDEX_MAP[] = {0, 2, 1, 3};

    /* Calculate chip position in virtual matrix */
    size_t chip_index = (row / 8) + (col / 8) * 2;
    /* Apply physical layout mapping */
    chip_index = CHIP_INDEX_MAP[chip_index % 4] + (chip_index - chip_index % 4);
    uint8_t local_row = row % 8;
    uint8_t local_col = col % 8;

    DrawPixel(chip_index, local_row, local_col, on);
  }

  /* Refresh display with current buffer */
  void Refresh() {
    mutex_.lock();
    for (uint8_t row = 0; row < 8; ++row) {
      std::array<uint8_t, N> regs;
      std::array<uint8_t, N> data;
      regs.fill(REG_DIGIT0 + row);
      for (size_t i = 0; i < N; ++i) {
        data[i] = framebuffer_[i][row];
      }
      WriteCommandRaw(regs, data);
    }
    mutex_.unlock();
  }

  /* Write to specific chip */
  void WriteToChip(size_t index, uint8_t addr, uint8_t data) {
    std::array<uint8_t, N> regs;
    std::array<uint8_t, N> data_all;
    regs.fill(REG_NOOP);
    data_all.fill(0x00);
    if (index >= N) {
      return;
    }
    regs[index] = addr;
    data_all[index] = data;
    WriteCommandRaw(regs, data_all);
  }

  /* Full diagnostic test pattern */
  void TestEachChip() {
    Clear();
    /* Draw left border */
    for (int i = 0; i < 16; ++i) {
      DrawPixelMatrix2(i, 0, true);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    /* Draw bottom border */
    for (int i = 0; i < 16; ++i) {
      DrawPixelMatrix2(15, i, true);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    /* Draw middle vertical line */
    for (int i = 0; i < 16; ++i) {
      DrawPixelMatrix2(i, 16, true);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    /* Draw right border */
    for (int i = 16; i < 32; ++i) {
      DrawPixelMatrix2(15, i, true);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    /* Draw right vertical line */
    for (int i = 15; i > 0; --i) {
      DrawPixelMatrix2(i, 31, true);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    /* Draw top right border */
    for (int i = 31; i > 16; --i) {
      DrawPixelMatrix2(0, i, true);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    /* Draw middle horizontal line */
    for (int i = 15; i > 0; --i) {
      DrawPixelMatrix2(i, 15, true);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    /* Draw top left border */
    for (int i = 15; i > 0; --i) {
      DrawPixelMatrix2(0, i, true);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  
  void Lock(){
    mutex_.lock();
  }
  
  void Unlock(){
    mutex_.unlock();
  }
  
  void SetLight(uint8_t light){
    SetIntensity(light);
  }

 private:
  // NOLINTNEXTLINE
  SpiDevice& spi_;
  Gpio* cs_;
  std::array<std::array<uint8_t, 8>, N> framebuffer_;
  std::thread thread_; /* Thread */
  std::mutex mutex_;
  
  /* Write to all chips with same register */
  void WriteAll(uint8_t addr, uint8_t value) {
    std::array<uint8_t, N> data;
    data.fill(value);
    WriteCommand(addr, data);
  }

  /* Command write wrapper */
  void WriteCommand(uint8_t addr, const std::array<uint8_t, N>& data) {
    std::array<uint8_t, N> regs;
    regs.fill(addr);
    WriteCommandRaw(regs, data);
  }

  /* Low-level SPI write operation */
  void WriteCommandRaw(const std::array<uint8_t, N>& regs,
                       const std::array<uint8_t, N>& data) {
    std::array<uint8_t, N * 2> tx_buf{};
    for (size_t i = 0; i < N; ++i) {
      const size_t HW_INDEX = N - 1 - i;
      tx_buf[i * 2] = regs[HW_INDEX];
      tx_buf[i * 2 + 1] = data[HW_INDEX];
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
#ifndef TEST_BUILD
    if (ioctl(spi_.Fd(), SPI_IOC_MESSAGE(1), &transfer) < 0) {
      cs_->Write(1);
      std::perror("SPI transfer failed");
    }
#endif

    usleep(100);
    cs_->Write(1);
    usleep(100);
  }
};
