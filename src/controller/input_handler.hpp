#pragma once

#include <functional>
#include <iostream>
#include <semaphore>

#include "bsp_pwm.hpp"
#include "comp_gui.hpp"
#include "mode_manager.hpp"

// Handles input from physical buttons, including mode switching,
// stopwatch toggle, and timer stop operations.
class InputHandler {
 public:
  // Initialize input handler with hardware references and logic callbacks.
  void Init(Gpio* btn1, Gpio* btn2, PWM* buzzer, CompGuiX* gui,
            ModeManager* mode_manager, std::function<void()> onStopwatchToggle,
            std::function<void()> onTimerStop) {
    mode_manager_ = mode_manager;
    gui_ = gui;
    buzzer_ = buzzer;
    onStopwatchToggle_ = std::move(onStopwatchToggle);
    onTimerStop_ = std::move(onTimerStop);

    // Configure Button 1 interrupt callback:
    // - Switches to next mode
    // - Disables sand animation (if any)
    // - Plays a short beep
    btn1->EnableInterruptRisingEdgeWithCallback([this]() {
      gpio_int_sem_1_.release();  // Release semaphore to indicate interrupt
      mode_manager_->NextMode();  // Cycle to the next mode
      gui_->SandDisable();        // Turn off sand effect
      std::cout << "Button 1\n";
      buzzer_->PlayNote(PWM::NoteName::C, 7, 50);  // Feedback sound
    });

    // Configure Button 2 interrupt callback:
    // - Plays a short beep
    // - Depending on the current mode:
    //     - In stopwatch mode: toggle start/stop
    //     - In timer mode: stop timer and disable sand animation
    btn2->EnableInterruptRisingEdgeWithCallback([this]() {
      gpio_int_sem_2_.release();  // Release semaphore to indicate interrupt
      std::cout << "Button 2\n";
      buzzer_->PlayNote(PWM::NoteName::C, 7, 50);  // Feedback sound

      auto mode = mode_manager_->GetMode();
      if (mode == ModeManager::Mode::STOPWATCH) {
        onStopwatchToggle_();  // Toggle stopwatch (start/stop)
      } else if (mode == ModeManager::Mode::TIMER &&
                 mode_manager_->IsTimerRunning()) {
        onTimerStop_();       // Stop timer
        gui_->SandDisable();  // Stop sand animation
      }
    });
  }

 private:
  ModeManager* mode_manager_ = nullptr;  // Pointer to mode manager
  CompGuiX* gui_ = nullptr;              // Pointer to GUI handler
  PWM* buzzer_ = nullptr;                // Pointer to PWM buzzer

  std::function<void()> onStopwatchToggle_;  // Callback for stopwatch control
  std::function<void()> onTimerStop_;        // Callback for stopping timer

  // Semaphores for tracking button interrupts (optional use in threaded
  // context)
  std::binary_semaphore gpio_int_sem_1_{0};
  std::binary_semaphore gpio_int_sem_2_{0};
};
