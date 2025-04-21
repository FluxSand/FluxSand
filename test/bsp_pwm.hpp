
#pragma once
#include <iostream>
#include <cmath>
#include <string>

class PWM {
 public:
  enum class NoteName { C = 0, Cs, D, Ds, E, F, Fs, G, Gs, A, As, B };

  PWM(int channel, int frequency_hz = 1000, float duty_percent = 0.0f, int chip = 2)
      : frequency_hz_(frequency_hz), duty_percent_(duty_percent) {}

  void SetFrequency(uint32_t hz) { frequency_hz_ = hz; }
  void SetDutyCycle(float percent) { duty_percent_ = percent; }
  void Enable() { enabled_ = true; }
  void Disable() { enabled_ = false; }

  void Beep(uint32_t freq, uint32_t duration_ms) {
    std::cout << "[TEST] Beep: " << freq << " Hz for " << duration_ms << " ms\n";
  }

  void PlayNote(NoteName note, uint32_t octave, uint32_t duration_ms) {
    float midi = static_cast<float>(note) + (octave + 1.0f) * 12.0f;
    float freq = 440.0f * std::pow(2.0f, (midi - 69.0f) / 12.0f);
    Beep(static_cast<uint32_t>(freq), duration_ms);
  }

 private:
  int frequency_hz_;
  float duty_percent_;
  bool enabled_ = false;
};
