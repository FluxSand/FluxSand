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
      mode_ = static_cast<Mode>((static_cast<int>(mode_) + 1) %
                                static_cast<int>(Mode::MODE_NUM));
      std::cout << "Button 1\n";
    });

    gpio_user_button_2->EnableInterruptRisingEdgeWithCallback([this]() {
      gpio_int_sem_2_.release();
      std::cout << "Button 2\n";

      if (mode_ == Mode::STOPWATCH) {
        if (stopwatch_running_) {
          StopStopwatch();
        } else {
          StartStopwatch();
        }
      }
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

      static std::deque<float> light_queue;
      light_queue.push_front(lux);
      if (light_queue.size() > 250) {
        auto avg = 0.0f;
        for (auto l : light_queue) {
          avg += l;
        }
        avg /= static_cast<float>(light_queue.size());
        light_ = avg;
        static int counter = 0;
        if (counter > 20) {
          gui_->SetLight(static_cast<uint8_t>(light_ / 20 + 1));
          counter = 0;
        } else {
          counter++;
        }
        light_queue.pop_back();
      }
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

    switch (mode_) {
      case Mode::TIME_LANDSCAPE:
        gui_->RenderTimeLandscape(hour, minute);
        break;
      case Mode::TIME_PORTRAIT:
        gui_->RenderTimePortrait(hour, minute);
        break;
      case Mode::HUMIDITY:
        gui_->RenderHumidity(static_cast<uint8_t>(aht20_->GetHumidity()));
        break;
      case Mode::TEMPERATURE:
        gui_->RenderTemperature(static_cast<uint8_t>(temperature_));
        break;
      case Mode::STOPWATCH: {
        int64_t display_sec = stopwatch_elapsed_sec_;
        if (stopwatch_running_) {
          auto now = std::chrono::steady_clock::now();
          display_sec += std::chrono::duration_cast<std::chrono::seconds>(
                             now - stopwatch_start_time_)
                             .count();
        }
        gui_->RenderTimeLandscape(display_sec / 60 % 24, display_sec % 60);
        break;
      }
      default:
        std::cout << "Unknown mode\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  void StartStopwatch() {
    if (!stopwatch_running_) {
      stopwatch_start_time_ = std::chrono::steady_clock::now();
      stopwatch_running_ = true;
      mode_ = Mode::STOPWATCH;
      std::cout << "Stopwatch started\n";
    }
  }

  void StopStopwatch() {
    if (stopwatch_running_) {
      auto now = std::chrono::steady_clock::now();
      stopwatch_elapsed_sec_ +=
          std::chrono::duration_cast<std::chrono::seconds>(
              now - stopwatch_start_time_)
              .count();
      stopwatch_running_ = false;
      std::cout << "Stopwatch stopped\n";
    }
  }

  enum class Mode : uint8_t {
    TIME_LANDSCAPE,
    TIME_PORTRAIT,
    HUMIDITY,
    TEMPERATURE,
    STOPWATCH,
    MODE_NUM
  };

  Mode mode_ = Mode::HUMIDITY;

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

  std::chrono::steady_clock::time_point timer_start_time_;
  int timer_duration_sec_ = 0;
  bool timer_active_ = false;

  bool stopwatch_running_ = false;
  std::chrono::steady_clock::time_point stopwatch_start_time_;
  int64_t stopwatch_elapsed_sec_ = 0;

  ModelOutput inference_result_ = ModelOutput::UNRECOGNIZED;
};
