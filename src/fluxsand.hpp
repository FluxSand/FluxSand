#include <chrono>
#include <ctime>
#include <iostream>

#include "ads1115.hpp"
#include "aht20.hpp"
#include "bmp280.hpp"
#include "bsp_pwm.hpp"
#include "comp_ahrs.hpp"
#include "comp_gui.hpp"
#include "comp_inference.hpp"
#include "max7219.hpp"

class FluxSand {
 public:
  FluxSand(PWM* pwm_buzzer, Gpio* gpio_user_button_1, Gpio* gpio_user_button_2,
           CompGuiX* gui, Bmp280* bmp280, Aht20* aht20, Ads1115<2>* ads1115,
           AHRS* ahrs, InferenceEngine* inference)
      : pwm_buzzer_(pwm_buzzer),
        gpio_user_button_1_(gpio_user_button_1),
        gpio_user_button_2_(gpio_user_button_2),
        gui_(gui),
        bmp280_(bmp280),
        aht20_(aht20),
        ads1115_(ads1115),
        ahrs_(ahrs),
        inference_(inference),
        gpio_int_sem_1_(0),
        gpio_int_sem_2_(0) {
    /* Reset buzzer */
    pwm_buzzer->SetDutyCycle(0.0f);
    pwm_buzzer->Enable();

    /* Register button interrupt callbacks */
    gpio_user_button_1->EnableInterruptRisingEdgeWithCallback([this]() {
      gpio_int_sem_1_.release();
      mode_ = Mode::TIME_LANDSCAPE;
      std::cout << "Button 1\n";
    });

    gpio_user_button_2->EnableInterruptRisingEdgeWithCallback([this]() {
      gpio_int_sem_2_.release();
      mode_ = Mode::TIME_PORTRAIT;
      std::cout << "Button 2\n";
    });

    ads1115->RegisterChannelCallback(0, [this](float voltage) {
      constexpr float VCC = 3.3f;
      constexpr float R_REF = 100000.0f;
      float r_ntc = R_REF * voltage / (VCC - voltage);

      constexpr float B = 3950.0f;
      constexpr float T0 = 298.15f;
      constexpr float R0 = 10000.0f;
      float temp = 1.0f / (1.0f / T0 + (1.0f / B) * log(r_ntc / R0)) - 273.15f;
      temperature_ = temp;
    });

    ads1115->RegisterChannelCallback(1, [this](float voltage) {
      constexpr float VCC = 3.3f;
      constexpr float R_REF = 100000.0f;
      float r_photo = R_REF * voltage / (VCC - voltage);
      constexpr float K = 1500000.0f;
      constexpr float GAMMA = 1.5f;
      float lux = K / pow(r_photo, GAMMA);
      light_ = lux;
    });

    inference->RegisterDataCallback([this](ModelOutput result) {
      inference_result_ = result;
      std::cout << "New Gesture: " << LABELS.find(result)->second << '\n';
    });
  }

  void Run() {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm* local_time = std::localtime(&t);
    int hour = local_time->tm_hour;
    int minute = local_time->tm_min;
    
    switch(mode_) {
      case Mode::TIME_LANDSCAPE:
        gui_->RenderTimeLandscape(hour, minute);
        break;
      case Mode::TIME_PORTRAIT:
        gui_->RenderTimePortrait(hour, minute);
        break;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  enum class Mode: uint8_t {
    TIME_LANDSCAPE,
    TIME_PORTRAIT
  };

  Mode mode_ = Mode::TIME_PORTRAIT;

  PWM* pwm_buzzer_;
  Gpio* gpio_user_button_1_;
  Gpio* gpio_user_button_2_;
  CompGuiX* gui_;
  Bmp280* bmp280_;
  Aht20* aht20_;
  Ads1115<2>* ads1115_;
  AHRS* ahrs_;
  InferenceEngine* inference_;

  std::binary_semaphore gpio_int_sem_1_, gpio_int_sem_2_;

  float temperature_, light_;

  ModelOutput inference_result_ = ModelOutput::UNRECOGNIZED;
};
