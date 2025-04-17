#pragma once

#include <array>
#include <string>
#include <unordered_map>

#include "max7219.hpp"

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

  explicit CompGuiX(Max7219<8>& display,
                    Orientation ori = Orientation::Portrait)
      : display_(display), orientation_(ori) {}

  void SetOrientation(Orientation ori) { orientation_ = ori; }

  void Draw(RegionID region, int value) {
    int tens = (value / 10) % 10;
    int ones = value % 10;

    auto [baseX, baseY] = REGION_OFFSETSS[static_cast<int>(region)];
    DrawDigit(baseX, baseY, FONT[tens], region);
    DrawDigit(baseX + 5, baseY, FONT[ones], region);  // Right-aligned
  }

  void Clear() { display_.Clear(); }

 private:
  // NOLINTNEXTLINE
  Max7219<8>& display_;
  Orientation orientation_;

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

  static constexpr std::array<std::pair<int, int>, 4> REGION_OFFSETSS = {{
      {-2, 1},  // SCREEN_0_LANDSCAPE
      {3, -3},  // SCREEN_1_LANDSCAPE
      {0, 1},   // SCREEN_0_PORTRAIT
      {0, 1},   // SCREEN_1_PORTRAIT
  }};

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

  void PlotRotated45(int lx, int ly, RegionID region, bool on) {
    int row = 0, col = 0;

    if (orientation_ != Orientation::Landscape) {
      // Portrait mode: 45° rotation from top-left to bottom-right
      row = lx + ly;
      col = -lx + ly;

      if (region == RegionID::SCREEN_0_PORTRAIT) {
        col += 8;
      } else if (region == RegionID::SCREEN_1_PORTRAIT) {
        col += 24;
      }

    } else {
      // Landscape mode: 45° rotation from bottom-left to top-right
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
  void RenderTimeLandscape(uint8_t hour, uint8_t minute) {
    display_.Lock();
    Clear();
    SetOrientation(Orientation::Landscape);
    display_.DrawPixel(3, 7, 4, true);  // Colon dot
    display_.DrawPixel(3, 4, 7, true);  // Colon dot
    Draw(RegionID::SCREEN_0_LANDSCAPE, hour);
    Draw(RegionID::SCREEN_1_LANDSCAPE, minute);
    display_.Unlock();
  }

  void RenderTimePortrait(uint8_t hour, uint8_t minute) {
    display_.Lock();
    Clear();
    SetOrientation(Orientation::Portrait);

    // Colon dots - right
    display_.DrawPixel(4, 2, 0, true);
    display_.DrawPixel(4, 3, 1, true);
    display_.DrawPixel(4, 4, 2, true);
    display_.DrawPixel(4, 2, 2, true);
    display_.DrawPixel(4, 0, 2, true);
    display_.DrawPixel(4, 1, 3, true);
    display_.DrawPixel(4, 2, 4, true);

    // Colon dots - left
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
};
