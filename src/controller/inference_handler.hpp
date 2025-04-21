#pragma once

#include "bsp_pwm.hpp"
#include "comp_gui.hpp"
#include "comp_inference.hpp"
#include "mode_manager.hpp"

// Handles gesture inference results and maps them to application actions.
// Uses callbacks and mode state to update GUI and behavior.
class InferenceHandler {
 public:
  // Initializes the inference handler with required components and callbacks.
  void Init(InferenceEngine* inference, ModeManager* mode_manager,
            CompGuiX* gui, PWM* buzzer,
            std::function<void(int)> startTimerCallback,
            std::function<void()> stopTimerCallback) {
    mode_manager_ = mode_manager;
    gui_ = gui;
    buzzer_ = buzzer;
    startTimerCallback_ = std::move(startTimerCallback);
    stopTimerCallback_ = std::move(stopTimerCallback);

    // Register a callback to receive gesture inference results
    inference->RegisterDataCallback([this](ModelOutput result) {
      std::cout << "New Gesture: " << LABELS.find(result)->second << '\n';

      // Play a confirmation beep when a gesture is detected
      buzzer_->PlayNote(PWM::NoteName::C, 7, 300);

      // Get current mode and orientation
      auto mode = mode_manager_->GetMode();
      bool landscape = mode_manager_->IsLandscape();

      // Handle each gesture type and map to corresponding behavior
      switch (result) {
        case ModelOutput::TILT_RIGHT:
          // In timer mode (not running), increase timer duration
          if (mode == ModeManager::Mode::TIMER &&
              !mode_manager_->IsTimerRunning()) {
            mode_manager_->AdjustTimer(+300);
          }
          // In time mode (portrait), switch to landscape
          else if (mode == ModeManager::Mode::TIME && !landscape) {
            mode_manager_->SetLandscape(true);
          }
          // In timer mode (portrait and running), switch to landscape and
          // disable sand animation
          else if (mode == ModeManager::Mode::TIMER && !landscape &&
                   mode_manager_->IsTimerRunning()) {
            mode_manager_->SetLandscape(true);
            gui_->SandDisable();
          }
          break;

        case ModelOutput::TILT_LEFT:
          // In timer mode (not running), decrease timer duration
          if (mode == ModeManager::Mode::TIMER &&
              !mode_manager_->IsTimerRunning()) {
            mode_manager_->AdjustTimer(-300);
          }
          // In time mode (landscape), switch to portrait
          else if (mode == ModeManager::Mode::TIME && landscape) {
            mode_manager_->SetLandscape(false);
          }
          // In timer mode (landscape and running), switch to portrait and
          // enable sand animation
          else if (mode == ModeManager::Mode::TIMER && landscape &&
                   mode_manager_->IsTimerRunning()) {
            mode_manager_->SetLandscape(false);
            gui_->SandEnable();
          }
          break;

        case ModelOutput::SHAKE_FORWARD:
          // Start timer if in timer mode and not already running
          if (mode == ModeManager::Mode::TIMER &&
              !mode_manager_->IsTimerRunning()) {
            startTimerCallback_(mode_manager_->GetRemainingTimerSeconds());
          }
          break;

        case ModelOutput::SHAKE_BACKWARD:
          // Stop timer and disable sand animation if in timer mode
          if (mode == ModeManager::Mode::TIMER) {
            stopTimerCallback_();
            gui_->SandDisable();
          }
          break;

        default:
          break;
      }
    });
  }

 private:
  ModeManager* mode_manager_ = nullptr;          // Mode state handler
  CompGuiX* gui_ = nullptr;                      // GUI interface
  PWM* buzzer_ = nullptr;                        // Buzzer interface
  std::function<void(int)> startTimerCallback_;  // Callback to start timer
  std::function<void()> stopTimerCallback_;      // Callback to stop timer
};
