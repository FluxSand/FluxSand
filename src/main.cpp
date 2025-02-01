#include <format>
#include <iostream>
#include <thread>

#include "bsp_gpio.hpp"
#include "bsp_spi.hpp"
#include "comp_ahrs.hpp"
#include "mpu9250.hpp"
#include "om.hpp"

int main() {
  Message();

  SpiDevice spi_device("/dev/spidev0.0", 1000000, SPI_MODE_0);
  Gpio gpio_cs("gpiochip0", 14, true, 1);

  Mpu9250 mpu9250(&spi_device, &gpio_cs);

  AHRS ahrs;

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  return 0;
}
