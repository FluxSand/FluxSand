#pragma once

#include <chrono>
#include <iostream>

// Handles application modes and manages stopwatch/timer logic.
class ModeManager {
 public:
  // Enumeration of available UI modes.
  enum class Mode : uint8_t {
    TIME,         // Clock display
    HUMIDITY,     // Humidity display
    TEMPERATURE,  // Temperature display
    STOPWATCH,    // Stopwatch mode
    TIMER,        // Countdown timer mode
    MODE_NUM      // Total number of modes (used for cycling)
  };

  // Constructor: start in TIME mode.
  ModeManager() : mode_(Mode::TIME) {}

  // Cycle to the next mode in order (looping back after the last one).
  void NextMode() {
    mode_ = static_cast<Mode>((static_cast<int>(mode_) + 1) %
                              static_cast<int>(Mode::MODE_NUM));
  }

  // Get the current mode.
  Mode GetMode() const { return mode_; }

  // Set or query display orientation (portrait/landscape).
  void SetLandscape(bool val) { landscape_ = val; }
  bool IsLandscape() const { return landscape_; }

  // --- Stopwatch Functionality ---

  // Start the stopwatch if not already running.
  void StartStopwatch() {
    if (!stopwatch_running_) {
      stopwatch_start_time_ = std::chrono::steady_clock::now();
      stopwatch_running_ = true;
      mode_ = Mode::STOPWATCH;
      std::cout << "Stopwatch started\n";
    }
  }

  // Stop the stopwatch and reset elapsed time.
  void StopStopwatch() {
    if (stopwatch_running_) {
      stopwatch_running_ = false;
      stopwatch_elapsed_sec_ = 0;
      std::cout << "Stopwatch stopped\n";
    }
  }

  // Get the number of seconds elapsed on the stopwatch.
  int64_t GetStopwatchSeconds() const {
    int64_t display_sec = stopwatch_elapsed_sec_;
    if (stopwatch_running_) {
      auto now = std::chrono::steady_clock::now();
      display_sec += std::chrono::duration_cast<std::chrono::seconds>(
                         now - stopwatch_start_time_)
                         .count();
    }
    return display_sec;
  }

  // Check if the stopwatch is currently running.
  bool IsStopwatchRunning() const { return stopwatch_running_; }

  // --- Timer Functionality ---

  // Start the countdown timer for a given duration (in seconds).
  void StartTimer(int duration_sec) {
    timer_duration_sec_ = duration_sec;
    max_duration_sec_ = duration_sec;
    timer_start_time_ = std::chrono::steady_clock::now();
    timer_active_ = true;
    mode_ = Mode::TIMER;
    std::cout << "Timer started for " << duration_sec << " seconds\n";
  }

  // Stop the timer immediately and reset its duration.
  void StopTimer() {
    timer_active_ = false;
    timer_duration_sec_ = 0;
    std::cout << "Timer stopped\n";
  }

  // Get remaining time (in seconds). Auto-stops when expired.
  int64_t GetRemainingTimerSeconds() {
    int64_t remaining = timer_duration_sec_;
    if (timer_active_) {
      auto now = std::chrono::steady_clock::now();
      int64_t elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                            now - timer_start_time_)
                            .count();
      if (elapsed >= timer_duration_sec_) {
        remaining = 0;
        timer_active_ = false;
        std::cout << "Timer finished\n";
      } else {
        remaining -= elapsed;
      }
    }
    return remaining;
  }

  // Check if the timer is currently active.
  bool IsTimerRunning() const { return timer_active_; }

  // Get the maximum duration the timer was set to (used for animation ratio).
  int GetMaxTimerDuration() const { return max_duration_sec_; }

  // Adjust timer duration (before starting), with clamping.
  void AdjustTimer(int delta_sec) {
    if (!timer_active_) {
      timer_duration_sec_ = std::clamp(timer_duration_sec_ + delta_sec, 0,
                                       60 * 99 - 1);  // max 99 minutes
    }
  }

 private:
  Mode mode_;               // Current display mode
  bool landscape_ = false;  // Display orientation

  // Stopwatch state
  bool stopwatch_running_ = false;
  std::chrono::steady_clock::time_point stopwatch_start_time_;
  int64_t stopwatch_elapsed_sec_ = 0;

  // Timer state
  bool timer_active_ = false;
  std::chrono::steady_clock::time_point timer_start_time_;
  int timer_duration_sec_ = 0;
  int max_duration_sec_ = 0;
};
