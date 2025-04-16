#include <format>
#include <iostream>
#include <thread>

#include "aht20.hpp"
#include "bmp280.hpp"
#include "bsp_gpio.hpp"
#include "bsp_i2c.hpp"
#include "bsp_pwm.hpp"
#include "bsp_spi.hpp"
#include "comp_ahrs.hpp"
#include "comp_inference.hpp"
#include "libxr.hpp"
#include "max7219.hpp"
#include "message.hpp"
#include "mpu9250.hpp"

int main() {
  LibXR::PlatformInit();

  PWM pwm_buzzer(0, 50, 7.5);

  pwm_buzzer.PlayNote(PWM::NoteName::C, 7, 300);
  pwm_buzzer.PlayNote(PWM::NoteName::D, 7, 300);
  pwm_buzzer.PlayNote(PWM::NoteName::E, 7, 300);

  pwm_buzzer.Disable();

  SpiDevice spi_display("/dev/spidev1.0", 1000000, SPI_MODE_0);
  Gpio gpio_display_cs("gpiochip0", 26, true, 1);

  Max7219<8> display(spi_display, &gpio_display_cs);

  I2cDevice i2c("/dev/i2c-1", Bmp280::DEFAULT_I2C_ADDR);
  Bmp280 bmp(i2c);
  I2cDevice i2c_1("/dev/i2c-1", Aht20::DEFAULT_I2C_ADDR);
  Aht20 aht20(i2c_1);

  SpiDevice spi_imu_device("/dev/spidev0.0", 1000000, SPI_MODE_0);
  Gpio gpio_imu_cs("gpiochip0", 22, true, 1);
  Gpio gpio_imu_int("gpiochip0", 27, false, 1);

  Mpu9250 mpu9250(&spi_imu_device, &gpio_imu_cs, &gpio_imu_int);

  AHRS ahrs;

  auto ramfs = LibXR::RamFS();

  InferenceEngine inference_engine(ONNX_MODEL_PATH, 0.05f, 0.75f, 20, 6);

  LibXR::Terminal<128, 128, 10, 20> terminal(ramfs);

  ramfs.Add(inference_engine.GetFile());

  std::thread terminal_thread([&terminal]() { terminal.ThreadFun(&terminal); });

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
