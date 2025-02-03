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
  Gpio gpio_cs("gpiochip0", 14, true, 1);

  Mpu9250 mpu9250(&spi_device, &gpio_cs);

  AHRS ahrs;

  InferenceEngine inference_engine(ONNX_MODEL_PATH, 0.1f);

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
