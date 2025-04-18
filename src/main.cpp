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
#include "comp_gui.hpp"
#include "comp_inference.hpp"
#include "fluxsand.hpp"
#include "max7219.hpp"
#include "mpu9250.hpp"

int main() {
  /* Buzzer */
  PWM pwm_buzzer(0, 50, 7.5);

  pwm_buzzer.PlayNote(PWM::NoteName::C, 7, 250);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  pwm_buzzer.PlayNote(PWM::NoteName::D, 7, 250);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  pwm_buzzer.PlayNote(PWM::NoteName::E, 7, 250);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  pwm_buzzer.Disable();

  /* User button */
  Gpio gpio_user_button_1("gpiochip0", 23, false, 1);
  Gpio gpio_user_button_2("gpiochip0", 24, false, 1);

  /* Max7219 display */
  SpiDevice spi_display("/dev/spidev1.0", 1000000, SPI_MODE_0);
  Gpio gpio_display_cs("gpiochip0", 26, true, 1);
  Max7219<8> display(spi_display, &gpio_display_cs);

  /* BMP280 and AHT20 */
  I2cDevice i2c_bmp280("/dev/i2c-1", Bmp280::DEFAULT_I2C_ADDR);
  Bmp280 bmp280(i2c_bmp280);

  I2cDevice i2c_aht20("/dev/i2c-1", Aht20::DEFAULT_I2C_ADDR);
  Aht20 aht20(i2c_aht20);

  /* ADS1115 */
  I2cDevice i2c_mpu9250("/dev/i2c-0", Ads1115<2>::DEFAULT_I2C_ADDR);
  Gpio gpio_ads1115_int("gpiochip0", 5, false, 1);
  Ads1115<2> ads1115(i2c_mpu9250, gpio_ads1115_int);

  /* MPU9250 */
  SpiDevice spi_imu_device("/dev/spidev0.0", 1000000, SPI_MODE_0);
  Gpio gpio_imu_cs("gpiochip0", 22, true, 1);
  Gpio gpio_imu_int("gpiochip0", 27, false, 1);
  Mpu9250 mpu9250(&spi_imu_device, &gpio_imu_cs, &gpio_imu_int);

  /* Orientation prediction and CNN model inference */
  AHRS ahrs;
  mpu9250.RegisterDataCallback(std::bind(
      &AHRS::OnData, &ahrs, std::placeholders::_1, std::placeholders::_2));

  InferenceEngine inference_engine(ONNX_MODEL_PATH, 0.01f, 0.7f, 15, 3);
  ahrs.RegisterDataCallback(std::bind(
      &InferenceEngine::OnData, &inference_engine, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3));

  CompGuiX gui(display);

  gui.Clear();

  /* Main loop */
  FluxSand fluxsand(&pwm_buzzer, &gpio_user_button_1, &gpio_user_button_2, &gui,
                    &bmp280, &aht20, &ads1115, &ahrs, &inference_engine);

  while (true) {
    fluxsand.Run();
  }

  return 0;
}
