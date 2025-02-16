#include <format>
#include <iostream>
#include <thread>

#include "bsp_gpio.hpp"
#include "bsp_spi.hpp"
#include "comp_ahrs.hpp"
#include "comp_inference.hpp"
#include "libxr.hpp"
#include "message.hpp"
#include "mpu9250.hpp"

int main() {
  LibXR::PlatformInit();

  SpiDevice spi_device("/dev/spidev0.0", 1000000, SPI_MODE_0);
  Gpio gpio_cs("gpiochip0", 23, true, 1);

  Mpu9250 mpu9250(&spi_device, &gpio_cs);

  AHRS ahrs;

  auto ramfs = LibXR::RamFS();

  InferenceEngine inference_engine(ONNX_MODEL_PATH, 0.2f);

  LibXR::Terminal<128, 128, 10, 20> terminal(ramfs);

  ramfs.Add(inference_engine.GetFile());

  std::thread terminal_thread([&terminal]() { terminal.ThreadFun(&terminal); });

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
