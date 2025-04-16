#include <format>
#include <iostream>
#include <thread>

#include "ads1115.hpp"
#include "aht20.hpp"
#include "bmp280.hpp"
#include "bsp_gpio.hpp"
#include "bsp_i2c.hpp"
#include "bsp_pwm.hpp"
#include "bsp_spi.hpp"
#include "comp_ahrs.hpp"
#include "comp_inference.hpp"
#include "max7219.hpp"
#include "mpu9250.hpp"

int main() {
  PWM pwm_buzzer(0, 50, 7.5);

  pwm_buzzer.PlayNote(PWM::NoteName::C, 7, 300);
  pwm_buzzer.PlayNote(PWM::NoteName::D, 7, 300);
  pwm_buzzer.PlayNote(PWM::NoteName::E, 7, 300);

  pwm_buzzer.Disable();

  SpiDevice spi_display("/dev/spidev1.0", 1000000, SPI_MODE_0);
  Gpio gpio_display_cs("gpiochip0", 26, true, 1);

  Max7219<8> display(spi_display, &gpio_display_cs);

  I2cDevice i2c_bmp280("/dev/i2c-1", Bmp280::DEFAULT_I2C_ADDR);
  Bmp280 bmp(i2c_bmp280);
  I2cDevice i2c_aht20("/dev/i2c-1", Aht20::DEFAULT_I2C_ADDR);
  Aht20 aht20(i2c_aht20);
  I2cDevice i2c_mpu9250("/dev/i2c-0", Ads1115<2>::DEFAULT_I2C_ADDR);
  Gpio gpio_ads1115_int("gpiochip0", 5, false, 1);

  Ads1115<2> ads1115(i2c_mpu9250, gpio_ads1115_int);

  SpiDevice spi_imu_device("/dev/spidev0.0", 1000000, SPI_MODE_0);
  Gpio gpio_imu_cs("gpiochip0", 22, true, 1);
  Gpio gpio_imu_int("gpiochip0", 27, false, 1);

  Mpu9250 mpu9250(&spi_imu_device, &gpio_imu_cs, &gpio_imu_int);

  AHRS ahrs;

  InferenceEngine inference_engine(ONNX_MODEL_PATH, 0.05f, 0.75f, 20, 6);

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
