#pragma once

#include <array>
#include <cmath>
#include <random>
#include <utility>
#include <vector>

class SandGrid {
 public:
  static constexpr int SIZE = 16;
  static constexpr float PI = 3.14159265f;

  const std::array<std::array<bool, SIZE>, SIZE>& GetGrid() const {
    return grid_;
  }

  bool GetCell(int r, int c) const {
    return InBounds(r, c) ? grid_[r][c] : false;
  }

  void SetCell(int r, int c, bool val) {
    if (InBounds(r, c)) grid_[r][c] = val;
  }

  bool AddNewSand() {
    if (grid_[15][15] == false) {
      grid_[15][15] = true;
      return true;
    } else {
      return false;
    }
  }

  bool AddGrainNearExisting() {
    std::vector<std::pair<int, int>> candidates;
    for (int row = 0; row < SIZE; ++row) {
      for (int col = 0; col < SIZE; ++col) {
        if (grid_[row][col]) {
          for (int dr = -1; dr <= 1; ++dr) {
            for (int dc = -1; dc <= 1; ++dc) {
              if (dr == 0 && dc == 0) continue;
              int nr = row + dr;
              int nc = col + dc;
              if (InBounds(nr, nc) && !grid_[nr][nc]) {
                candidates.emplace_back(nr, nc);
              }
            }
          }
        }
      }
    }

    if (candidates.empty()) return false;
    std::uniform_int_distribution<int> dist(0, candidates.size() - 1);
    auto [r, c] = candidates[dist(rng_)];
    grid_[r][c] = true;
    return true;
  }

  void StepOnce(float gravity_deg) {
    // Rotate gravity by 225 degrees to align with visual direction
    gravity_deg += 225.0f;
    if (gravity_deg >= 360.0f) gravity_deg -= 360.0f;

    // Convert gravity direction to unit vector (gx, gy)
    float angle_rad = gravity_deg * PI / 180.0f;
    float gx = std::cos(angle_rad);
    float gy = std::sin(angle_rad);

    // Define all 8 possible neighboring directions
    static const std::vector<std::pair<int, int>> directions = {
        {-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1},
    };

    // Random angle noise for more natural flow
    std::uniform_real_distribution<float> noise_dist(-30.0f, 30.0f);

    // List of sand moves to perform this frame: (from_row, from_col, to_row,
    // to_col)
    std::vector<std::tuple<int, int, int, int>> moves;

    // Tracks which cells are already targeted to avoid collision
    std::array<std::array<bool, SIZE>, SIZE> occupied_next = {};

    // ==== Custom row-wise center-outward traversal (bottom-up) ====
    std::vector<std::pair<int, int>> traversal_order;
    for (int r = SIZE - 1; r >= 0; --r) {
      int center = SIZE / 2;
      traversal_order.emplace_back(r, center);
      for (int offset = 1; offset < SIZE; ++offset) {
        int left = center - offset;
        int right = center + offset;
        if (left >= 0) traversal_order.emplace_back(r, left);
        if (right < SIZE) traversal_order.emplace_back(r, right);
      }
    }

    // ==== Traverse and decide moves ====
    for (auto [r, c] : traversal_order) {
      if (!grid_[r][c]) continue;  // Skip if no sand

      float noise_deg = noise_dist(rng_);
      float cos_threshold = std::cos((55.0f + noise_deg) * PI / 180.0f);

      std::pair<int, int> best_move = {0, 0};
      float best_dot = -2.0f;

      // Try all directions to find the best move
      for (auto [dr, dc] : directions) {
        int nr = r + dr;
        int nc = c + dc;
        if (!InBounds(nr, nc) || grid_[nr][nc])
          continue;  // Skip if out of bounds or not empty

        float vx = static_cast<float>(dc);
        float vy = static_cast<float>(dr);
        float len = std::sqrt(vx * vx + vy * vy);
        if (len == 0.0f) continue;

        vx /= len;
        vy /= len;

        // Dot product with gravity vector
        float dot = vx * gx + vy * gy;

        // Select move if direction is closer to gravity and exceeds threshold
        if (dot > cos_threshold && dot > best_dot) {
          best_dot = dot;
          best_move = {dr, dc};
        }
      }

      // Queue the move if a good direction was found and not already taken
      if (best_dot > -1.0f) {
        int nr = r + best_move.first;
        int nc = c + best_move.second;

        if (!occupied_next[nr][nc]) {
          occupied_next[nr][nc] = true;
          moves.emplace_back(r, c, nr, nc);
        }
      }
    }

    // ==== Perform all queued moves ====
    for (auto [r, c, nr, nc] : moves) {
      grid_[r][c] = false;
      grid_[nr][nc] = true;
    }

    // Optional: Debug output
    // std::cout << "[StepOnce] Moved " << moves.size() << " sand particles\n";
  }

  void Clear() {
    for (auto& row : grid_) {
      row.fill(false);
    }
  }

  int Count() const {
    int total = 0;
    for (const auto& row : grid_) {
      for (bool v : row) {
        total += v;
      }
    }
    return total;
  }

  static bool MoveSand(SandGrid* up, SandGrid* down, float angle) {
    if (angle < 90 || angle > 270) {
      if (up->grid_[0][0] && !down->grid_[15][15]) {
        down->grid_[15][15] = true;
        up->grid_[0][0] = false;
        return true;
      }
    } else {
      if (!up->grid_[0][0] && down->grid_[15][15]) {
        up->grid_[0][0] = true;
        down->grid_[15][15] = false;
        return true;
      }
    }
    return false;
  }

  void RunUnitTest() {
    std::cout << "[SandGrid::UnitTest] Starting sand grid test...\n";

    SandGrid test;
    test.Clear();

    // Test AddNewSand
    bool added = test.AddNewSand();
    std::cout << std::format("[Test] AddNewSand → {}\n",
                             added ? "✅ Success" : "❌ Failed");

    int before = test.Count();
    test.StepOnce(0.0f);
    int after = test.Count();
    std::cout << std::format("[Test] StepOnce → Particle count: {} → {}\n",
                             before, after);

    // Test MoveSand (grid transfer)
    SandGrid up, down;
    up.Clear();
    down.Clear();
    up.SetCell(0, 0, true);
    bool moved = SandGrid::MoveSand(&up, &down, 45.0f);
    std::cout << std::format("[Test] MoveSand(→down) → {}\n",
                             moved ? "✅ Success" : "❌ Failed");

    // Test Clear
    test.Clear();
    std::cout << std::format("[Test] Clear → Count after clear: {}\n",
                             test.Count());

    // StepOnce performance test
    std::cout << "[Perf] Running StepOnce 100 times...\n";
    for (int i = 0; i < 50; ++i) {
      test.AddGrainNearExisting();  // Fill a bit
    }

    std::vector<float> times;
    times.reserve(100);
    for (int i = 0; i < 100; ++i) {
      auto start = std::chrono::high_resolution_clock::now();
      test.StepOnce(0.0f);
      auto end = std::chrono::high_resolution_clock::now();
      float ms = std::chrono::duration<float, std::micro>(end - start).count();
      times.push_back(ms);
    }

    auto [min_it, max_it] = std::minmax_element(times.begin(), times.end());
    float avg =
        std::accumulate(times.begin(), times.end(), 0.0f) / times.size();

    std::cout << std::format(
        "[Perf] StepOnce timing (µs): min = {:>6.2f}, max = {:>6.2f}, avg = "
        "{:>6.2f}\n",
        *min_it, *max_it, avg);

    std::cout << "[SandGrid::UnitTest] ✅ Test complete.\n";
  }

 private:
  bool InBounds(int r, int c) const {
    return r >= 0 && r < SIZE && c >= 0 && c < SIZE;
  }

  std::array<std::array<bool, SIZE>, SIZE> grid_ = {};

  std::mt19937 rng_{std::random_device{}()};
};
