#pragma once

#include <array>
#include <semaphore>
#include <string>
#include <unordered_map>

#include "comp_sand.hpp"
#include "max7219.hpp"

/**
 * @brief LED Matrix GUI Controller with Dual Orientation Support
 *        and Sand Animation Physics
 */
class CompGuiX {
 public:
  // NOLINTNEXTLINE
  enum class Orientation { Landscape, Portrait };

  // NOLINTNEXTLINE
  enum class RegionID {
    SCREEN_0_LANDSCAPE,
    SCREEN_1_LANDSCAPE,
    SCREEN_0_PORTRAIT,
    SCREEN_1_PORTRAIT
  };

  /**
   * @brief Construct with MAX7219 display reference
   * @param display Reference to 8x8 LED matrix driver
   * @param ori Initial display orientation
   */
  explicit CompGuiX(Max7219<8>& display,
                    Orientation ori = Orientation::Portrait)
      : display_(display), orientation_(ori) {
    thread_ = std::thread(&CompGuiX::ThreadFun, this);
  }

  /// Update display orientation
  void SetOrientation(Orientation ori) { orientation_ = ori; }

  /**
   * @brief Draw 2-digit number in specified region
   * @param region Target display region
   * @param value Number to display (0-99)
   */
  void Draw(RegionID region, int value) {
    int tens = (value / 10) % 10;
    int ones = value % 10;

    auto [baseX, baseY] = REGION_OFFSETSS[static_cast<int>(region)];
    DrawDigit(baseX, baseY, FONT[tens], region);
    DrawDigit(baseX + 5, baseY, FONT[ones], region);  // Right-aligned
  }

  /// Clear display buffer
  void Clear() { display_.Clear(); }

  /**
   * @brief Set gravity direction for sand physics
   * @param gravity_angle Input angle in radians
   */
  void SetGravityDegree(float gravity_angle) {
    float deg = fmodf(
        630.0f - gravity_angle * 180.0f / static_cast<float>(M_PI), 360.0f);
    gravity_deg_ = deg;
  }

  bool sand_enable_ = false;      ///< Sand animation toggle
  SandGrid grid_up_, grid_down_;  ///< Sand particle containers
  float gravity_deg_ = 0.0f;      ///< Gravity direction (degrees)

  /// Enable sand simulation
  void SandEnable() { sand_enable_ = true; }

  /// Disable sand simulation
  void SandDisable() { sand_enable_ = false; }

  /// Animation thread entry point
  void ThreadFun() {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  init:
    grid_up_.Clear();
    grid_down_.Clear();

    // Initial fill animation
    for (int i = 0; i < 128;) {
      if (grid_up_.AddNewSand()) {
        i++;
      }
      grid_up_.StepOnce(0);
      RenderHourglass(&grid_up_, &grid_down_);
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }

    // Settling phase
    for (int i = 0; i < 16; i++) {
      grid_up_.StepOnce(0);
      RenderHourglass(&grid_up_, &grid_down_);
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }

    // Main simulation loop
    while (1) {
      if (sand_enable_) {
        if (reset_) {
          reset_ = false;
          goto init;
        }
        grid_up_.StepOnce(gravity_deg_);
        grid_down_.StepOnce(gravity_deg_);
        RenderHourglass(&grid_up_, &grid_down_);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
  }

 private:
  // Hardware interface
  Max7219<8>& display_;      ///< LED matrix driver
  Orientation orientation_;  ///< Current orientation
  bool reset_ = false;       ///< Reset flag
  std::thread thread_;       ///< Animation thread

  // 7-segment font definitions
  static constexpr std::array<std::array<std::string, 7>, 10> FONT = {{
      {"01110", "10001", "10011", "10101", "11001", "10001", "01110"},  // 0
      {"00100", "01100", "00100", "00100", "00100", "00100", "01110"},  // 1
      {"01110", "10001", "00001", "00010", "00100", "01000", "11111"},  // 2
      {"01110", "10001", "00001", "00110", "00001", "10001", "01110"},  // 3
      {"00010", "00110", "01010", "10010", "11111", "00010", "00010"},  // 4
      {"11111", "10000", "11110", "00001", "00001", "10001", "01110"},  // 5
      {"00110", "01000", "10000", "11110", "10001", "10001", "01110"},  // 6
      {"11111", "00001", "00010", "00100", "01000", "01000", "01000"},  // 7
      {"01110", "10001", "10001", "01110", "10001", "10001", "01110"},  // 8
      {"01110", "10001", "10001", "01111", "00001", "00010", "01100"},  // 9
  }};

  // Region coordinate offsets
  static constexpr std::array<std::pair<int, int>, 4> REGION_OFFSETSS = {{
      {-2, 1},  // SCREEN_0_LANDSCAPE
      {3, -3},  // SCREEN_1_LANDSCAPE
      {0, 1},   // SCREEN_0_PORTRAIT
      {0, 1},   // SCREEN_1_PORTRAIT
  }};

  /**
   * @brief Render digit using 5x7 bitmap
   * @param leftX Leftmost X coordinate
   * @param topY Top Y coordinate
   * @param bitmap Font glyph data
   * @param region Target display region
   */
  void DrawDigit(int leftX, int topY, const std::array<std::string, 7>& bitmap,
                 RegionID region) {
    for (int dy = 0; dy < 7; ++dy) {
      for (int dx = 0; dx < 5; ++dx) {
        if (bitmap[6 - dy][dx] == '1') {
          PlotRotated45(leftX + dx, topY + dy, region, true);
        }
      }
    }
  }

  /**
   * @brief Coordinate transformation for 45° rotated displays
   * @param lx Logical X coordinate
   * @param ly Logical Y coordinate
   * @param region Target display region
   * @param on Pixel state (true = on)
   */
  void PlotRotated45(int lx, int ly, RegionID region, bool on) {
    int row = 0, col = 0;

    if (orientation_ != Orientation::Landscape) {
      // Portrait mode transform
      row = lx + ly;
      col = -lx + ly;

      if (region == RegionID::SCREEN_0_PORTRAIT) {
        col += 8;
      } else if (region == RegionID::SCREEN_1_PORTRAIT) {
        col += 24;
      }
    } else {
      // Landscape mode transform
      row = lx - ly;
      col = lx + ly;

      if (region == RegionID::SCREEN_0_LANDSCAPE) {
        row += 8;
      } else if (region == RegionID::SCREEN_1_LANDSCAPE) {
        col += 16;
      }
    }

    if (row >= 0 && row < 16 && col >= 0 && col < 32) {
      display_.DrawPixelMatrix2(row, col, on);
    }
  }

 public:
  /// Set display brightness (0-15)
  void SetLight(uint8_t light) { display_.SetIntensity(light); }

  /// Render HH:MM in landscape orientation
  void RenderTimeLandscape(uint8_t hour, uint8_t minute) {
    display_.Lock();
    Clear();
    SetOrientation(Orientation::Landscape);
    display_.DrawPixel(3, 7, 4, true);  // Colon top
    display_.DrawPixel(3, 4, 7, true);  // Colon bottom
    Draw(RegionID::SCREEN_0_LANDSCAPE, hour);
    Draw(RegionID::SCREEN_1_LANDSCAPE, minute);
    display_.Unlock();
  }

  /// Render MM:SS with blinking colon
  void RenderTimeLandscapeMS(uint8_t minutes, uint8_t seconds) {
    display_.Lock();
    Clear();
    SetOrientation(Orientation::Landscape);
    if (seconds % 2 == 1) {
      display_.DrawPixel(3, 7, 4, true);
      display_.DrawPixel(3, 4, 7, true);
    }
    Draw(RegionID::SCREEN_0_LANDSCAPE, minutes);
    Draw(RegionID::SCREEN_1_LANDSCAPE, seconds);
    display_.Unlock();
  }

  /// Render HH:MM in portrait orientation
  void RenderTimePortrait(uint8_t hour, uint8_t minute) {
    display_.Lock();
    Clear();
    SetOrientation(Orientation::Portrait);

    // Complex colon design
    display_.DrawPixel(4, 2, 0, true);
    display_.DrawPixel(4, 3, 1, true);
    display_.DrawPixel(4, 4, 2, true);
    display_.DrawPixel(4, 2, 2, true);
    display_.DrawPixel(4, 0, 2, true);
    display_.DrawPixel(4, 1, 3, true);
    display_.DrawPixel(4, 2, 4, true);

    display_.DrawPixel(0, 4, 0, true);
    display_.DrawPixel(0, 4, 1, true);
    display_.DrawPixel(0, 4, 2, true);
    display_.DrawPixel(0, 3, 2, true);
    display_.DrawPixel(0, 2, 2, true);
    display_.DrawPixel(0, 2, 3, true);
    display_.DrawPixel(0, 2, 4, true);
    display_.DrawPixel(0, 1, 4, true);
    display_.DrawPixel(0, 0, 4, true);

    Draw(RegionID::SCREEN_0_PORTRAIT, minute);
    Draw(RegionID::SCREEN_1_PORTRAIT, hour);
    display_.Unlock();
  }

  /// Render MM:SS with dynamic colon
  void RenderTimePortraitMS(uint8_t minutes, uint8_t seconds) {
    display_.Lock();
    Clear();
    SetOrientation(Orientation::Portrait);

    if (seconds % 2 == 1) {
      display_.DrawPixel(4, 4, 0, true);
      display_.DrawPixel(4, 4, 1, true);
      display_.DrawPixel(4, 4, 2, true);
      display_.DrawPixel(4, 3, 2, true);
      display_.DrawPixel(4, 2, 2, true);
      display_.DrawPixel(4, 2, 3, true);
      display_.DrawPixel(4, 2, 4, true);
      display_.DrawPixel(4, 1, 4, true);
      display_.DrawPixel(4, 0, 4, true);
    }

    if (seconds % 2 != 1) {
      display_.DrawPixel(0, 0, 0, true);
      display_.DrawPixel(0, 1, 0, true);
      display_.DrawPixel(0, 1, 1, true);
      display_.DrawPixel(0, 1, 2, true);
      display_.DrawPixel(0, 2, 2, true);
    }

    Draw(RegionID::SCREEN_0_PORTRAIT, seconds);
    Draw(RegionID::SCREEN_1_PORTRAIT, minutes);
    display_.Unlock();
  }

  void RenderHumidity(uint8_t humidity) {
    static constexpr bool ICON[16][16] = {
        {0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 1, 1, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0},
        {0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
        {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
        {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    };
    display_.Lock();
    Clear();
    SetOrientation(Orientation::Portrait);
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        for (int k = 0; k < 8; k++) {
          for (int l = 0; l < 8; l++) {
            display_.DrawPixel(i * 2 + j + 4, k, l, ICON[k + i * 8][l + j * 8]);
          }
        }
      }
    }
    Draw(RegionID::SCREEN_0_PORTRAIT, humidity);
    display_.Unlock();
  }

  void RenderTemperature(uint8_t temperature) {
    static constexpr bool ICON[16][16] = {
        {0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
        {1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0},
        {1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0},
        {1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0},
        {0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0},
        {0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0},
        {0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0},
        {0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0},
        {0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0},
        {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0},
        {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    };
    display_.Lock();
    Clear();
    SetOrientation(Orientation::Portrait);
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        for (int k = 0; k < 8; k++) {
          for (int l = 0; l < 8; l++) {
            display_.DrawPixel(i * 2 + j + 4, k, l, ICON[k + i * 8][l + j * 8]);
          }
        }
      }
    }
    Draw(RegionID::SCREEN_0_PORTRAIT, temperature);
    display_.Unlock();
  }

  void RenderHourglass(SandGrid* up, SandGrid* down) {
    display_.Lock();
    Clear();
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        for (int k = 0; k < 8; k++) {
          for (int l = 0; l < 8; l++) {
            display_.DrawPixel(i * 2 + j + 4, k, l,
                               up->grid[k + i * 8][l + j * 8]);
          }
        }
      }
    }

    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        for (int k = 0; k < 8; k++) {
          for (int l = 0; l < 8; l++) {
            display_.DrawPixel(i * 2 + j, k, l,
                               down->grid[k + i * 8][l + j * 8]);
          }
        }
      }
    }
    display_.Unlock();
  }

  /// Reset sand simulation
  void Reset() { reset_ = true; }
};
