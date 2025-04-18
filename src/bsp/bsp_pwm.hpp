#pragma once

#include <unistd.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <semaphore>
#include <string>
#include <thread>

/**
 * @brief PWM driver with Beep and PlayNote functionality.
 *
 * This class controls PWM output on Raspberry Pi 5 using GPIO12 (channel 0) and
 * GPIO13 (channel 1).
 *
 * ⚠️ Requires enabling the following overlay in `/boot/firmware/config.txt`:
 *     dtoverlay=pwm-2chan
 */
class PWM {
 public:
  /**
   * @brief Musical note names used for MIDI note calculation.
   */
  // NOLINTNEXTLINE
  enum class NoteName { C = 0, Cs, D, Ds, E, F, Fs, G, Gs, A, As, B };

  /**
   * @brief Constructor: initializes PWM channel, sets initial frequency and
   * duty cycle.
   * @param channel PWM channel number (0 or 1).
   * @param frequency_hz Initial frequency in Hz. Default is 1000 Hz.
   * @param duty_percent Initial duty cycle (range: 0.0 ~ 1.0).
   * @param chip PWM chip number. Default is 2 for Raspberry Pi 5.
   */
  PWM(int channel, int frequency_hz = 1000, float duty_percent = 0.0f,
      int chip = 2)
      : channel_(channel), period_ns_(static_cast<int>(1e9 / frequency_hz)) {
    chip_path_ = "/sys/class/pwm/pwmchip" + std::to_string(chip);
    pwm_path_ = chip_path_ + "/pwm" + std::to_string(channel);

    // Export the PWM channel if it has not been exported yet.
    if (access(pwm_path_.c_str(), F_OK) != 0) {
      FILE* fp = fopen((chip_path_ + "/export").c_str(), "w");
      if (fp) {
        (void)(fprintf(fp, "%d", channel));
        (void)(fclose(fp));
        usleep(100000);  // Allow sysfs to initialize.
      }
    }

    WriteSysfs(pwm_path_ + "/period", period_ns_);
    SetDutyCycle(duty_percent);
    Enable();

    // Launch background thread to play notes asynchronously.
    note_thread_ = std::thread([this]() {
      while (true) {
        note_sem_.acquire();
        float midi = static_cast<float>(note_) +
                     (static_cast<float>(octave_) + 1.0f) * 12.0f;
        float freq = 440.0f * std::pow(2.0f, (midi - 69.0f) / 12.0f);
        Beep(static_cast<uint32_t>(freq), duration_ms_);
      }
    });
  }

  /**
   * @brief Destructor: disables PWM output.
   */
  ~PWM() { Disable(); }

  /**
   * @brief Set the output frequency of PWM.
   * @param hz Desired frequency in Hz.
   */
  void SetFrequency(uint32_t hz) {
    period_ns_ = static_cast<int>(1e9 / hz);
    WriteSysfs(pwm_path_ + "/period", period_ns_);
  }

  /**
   * @brief Set the duty cycle of PWM output.
   * @param percent Duty cycle as a float between 0.0 and 1.0.
   */
  void SetDutyCycle(float percent) {
    int duty_ns = static_cast<int>(percent * static_cast<float>(period_ns_));
    WriteSysfs(pwm_path_ + "/duty_cycle", duty_ns);
  }

  /**
   * @brief Enable the PWM output signal.
   */
  void Enable() { WriteSysfs(pwm_path_ + "/enable", 1); }

  /**
   * @brief Disable the PWM output signal.
   */
  void Disable() { WriteSysfs(pwm_path_ + "/enable", 0); }

  /**
   * @brief Output a beep with specific frequency and duration.
   * @param freq Frequency in Hz.
   * @param duration_ms Duration in milliseconds.
   */
  void Beep(uint32_t freq, uint32_t duration_ms) {
    SetFrequency(freq);
    SetDutyCycle(0.5f);
    Enable();
    usleep(duration_ms * 1000);
    Disable();
  }

  /**
   * @brief Play a musical note based on note name and octave.
   * @param note Note (C, D, E, etc.).
   * @param octave Octave number (e.g., 4 for A4 = 440 Hz).
   * @param duration_ms Duration of the note in milliseconds.
   */
  void PlayNote(NoteName note, uint32_t octave, uint32_t duration_ms) {
    note_ = note;
    octave_ = octave;
    duration_ms_ = duration_ms;
    note_sem_.release();
  }

  /**
   * @brief Set configuration parameters for alarm beep.
   * @param freq Alarm frequency in Hz.
   * @param duration_ms Duration of tone in milliseconds.
   * @param delay_ms Delay after the tone in milliseconds.
   */
  void SetAlarmConfig(uint32_t freq, uint32_t duration_ms, uint32_t delay_ms) {
    alarm_freq_ = freq;
    alarm_duration_ = duration_ms;
    alarm_delay_ = delay_ms;
  }

  /**
   * @brief Trigger an alarm sound followed by a delay.
   */
  void TriggerAlarm() {
    Beep(alarm_freq_, alarm_duration_);
    usleep(alarm_delay_ * 1000);
  }

 private:
  std::string chip_path_;  ///< Sysfs path to the PWM chip
  std::string pwm_path_;   ///< Sysfs path to the PWM channel
  int period_ns_;          ///< PWM period in nanoseconds
  int channel_;            ///< PWM channel number

  uint32_t alarm_freq_ = 1500;     ///< Default alarm frequency in Hz
  uint32_t alarm_duration_ = 300;  ///< Default alarm duration in ms
  uint32_t alarm_delay_ = 300;     ///< Default alarm delay in ms

  std::thread note_thread_;            ///< Background thread for playing notes
  NoteName note_;                      ///< Current note name
  uint32_t octave_;                    ///< Current octave
  uint32_t duration_ms_;               ///< Duration of note in ms
  std::binary_semaphore note_sem_{0};  ///< Semaphore to trigger note play

  /**
   * @brief Write an integer value to a sysfs file.
   * @param path Full path to the sysfs file.
   * @param value Integer value to write.
   * @return Write result (number of characters written) or -1 on error.
   */
  int WriteSysfs(const std::string& path, int value) const {
    FILE* fp = fopen(path.c_str(), "w");
    if (!fp) {
      return -1;
    }
    int result = fprintf(fp, "%d", value);
    (void)(fclose(fp));
    return result;
  }
};
