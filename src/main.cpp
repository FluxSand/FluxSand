#include <format>
#include <iostream>
#include <thread>

#include "gpio.hpp"
#include "mpu9250.hpp"
#include "spi.hpp"

int main() {
  try {
    spi::SpiDevice spi_device("/dev/spidev0.0", 5000000, SPI_MODE_0);
    gpio::Gpio gpio_cs("gpiochip0", 14, true, 1);
    imu::Mpu9250 mpu9250(&spi_device, &gpio_cs);

    mpu9250.Initialize();

    std::cout << "MPU9250 initialized. Starting data collection...\n";
    while (true) {
      mpu9250.ReadData();
      mpu9250.DisplayData();
      std::cout << "--------------------------------------\n";
      std::this_thread::sleep_for(
          std::chrono::milliseconds(1));  // 500ms sampling interval
    }

  } catch (const std::exception& e) {
    std::cerr << std::format("Exception: {}\n", e.what());
    return 1;
  }

  return 0;
}
