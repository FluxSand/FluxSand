#pragma once

#include "bsp.hpp"

namespace Type {

/**
 * @brief A class to represent and manipulate cyclic values (e.g., angles).
 *
 * This class ensures that values remain within a cyclic range, such as [0, 2π),
 * and provides arithmetic operations while preserving this property.
 */
class CycleValue {
 public:
  /**
   * @brief Default copy assignment operator.
   */
  CycleValue& operator=(const CycleValue&) = default;

  /**
   * @brief Normalizes a value to the range [0, 2π).
   *
   * @param value The input value (in radians).
   * @return The normalized value within [0, 2π).
   */
  static float Calculate(float value) {
    value = std::fmodf(value, M_2PI);
    if (value < 0) {
      value += M_2PI;
    }
    return value;
  }

  /**
   * @brief Constructs a CycleValue object from a float.
   *
   * @param value The input value.
   */
  CycleValue(const float& value) : value_(Calculate(value)) {}

  /**
   * @brief Constructs a CycleValue object from a double.
   *
   * @param value The input value.
   */
  CycleValue(const double& value)
      : value_(Calculate(static_cast<float>(value))) {}

  /**
   * @brief Copy constructor. Ensures the copied value remains within [0, 2π).
   *
   * @param value Another CycleValue object.
   */
  CycleValue(const CycleValue& value) : value_(value.value_) {
    while (value_ >= M_2PI) {
      value_ -= M_2PI;
    }

    while (value_ < 0) {
      value_ += M_2PI;
    }
  }

  /**
   * @brief Default constructor. Initializes the value to 0.
   */
  CycleValue() : value_(0.0f) {}

  /**
   * @brief Adds a float value and returns the resulting CycleValue.
   *
   * @param value The value to add.
   * @return A new CycleValue object.
   */
  CycleValue operator+(const float& value) {
    return CycleValue(value + value_);
  }

  /**
   * @brief Adds a double value and returns the resulting CycleValue.
   *
   * @param value The value to add.
   * @return A new CycleValue object.
   */
  CycleValue operator+(const double& value) {
    return CycleValue(static_cast<float>(value) + value_);
  }

  /**
   * @brief Adds another CycleValue and returns the result.
   *
   * @param value Another CycleValue object.
   * @return A new CycleValue object.
   */
  CycleValue operator+(const CycleValue& value) {
    return CycleValue(value.value_ + value_);
  }

  /**
   * @brief In-place addition of a float value.
   *
   * @param value The value to add.
   * @return A reference to this object.
   */
  CycleValue operator+=(const float& value) {
    value_ = Calculate(value + value_);
    return *this;
  }

  /**
   * @brief In-place addition of a double value.
   *
   * @param value The value to add.
   * @return A reference to this object.
   */
  CycleValue operator+=(const double& value) {
    value_ = Calculate(static_cast<float>(value) + value_);
    return *this;
  }

  /**
   * @brief In-place addition of another CycleValue.
   *
   * @param value Another CycleValue object.
   * @return A reference to this object.
   */
  CycleValue operator+=(const CycleValue& value) {
    value_ = Calculate(value.value_ + value_);
    return *this;
  }

  /**
   * @brief Computes the difference between this value and a float value.
   *
   * @param raw_value The value to subtract.
   * @return The difference normalized to [-π, π).
   */
  float operator-(const float& raw_value) {
    float value = Calculate(raw_value);
    float ans = value_ - value;
    while (ans >= M_PI) {
      ans -= M_2PI;
    }

    while (ans < -M_PI) {
      ans += M_2PI;
    }

    return ans;
  }

  /**
   * @brief Computes the difference between this value and a double value.
   *
   * @param raw_value The value to subtract.
   * @return The difference normalized to [-π, π).
   */
  float operator-(const double& raw_value) {
    float value = Calculate(static_cast<float>(raw_value));
    float ans = value_ - value;
    while (ans >= M_PI) {
      ans -= M_2PI;
    }

    while (ans < -M_PI) {
      ans += M_2PI;
    }

    return ans;
  }

  /**
   * @brief Computes the difference between this value and another CycleValue.
   *
   * @param value Another CycleValue object.
   * @return The difference normalized to [-π, π).
   */
  float operator-(const CycleValue& value) {
    float ans = value_ - value.value_;
    while (ans >= M_PI) {
      ans -= M_2PI;
    }

    while (ans < -M_PI) {
      ans += M_2PI;
    }

    return ans;
  }

  /**
   * @brief Negates the current value.
   *
   * @return A new CycleValue object.
   */
  CycleValue operator-() { return CycleValue(M_2PI - value_); }

  /**
   * @brief Implicit conversion to float.
   *
   * @return The normalized value as a float.
   */
  operator float() { return this->value_; }

  /**
   * @brief Assigns a float value.
   *
   * @param value The value to assign.
   * @return A reference to this object.
   */
  CycleValue& operator=(const float& value) {
    value_ = Calculate(value);
    return *this;
  }

  /**
   * @brief Assigns a double value.
   *
   * @param value The value to assign.
   * @return A reference to this object.
   */
  CycleValue& operator=(const double& value) {
    value_ = Calculate(static_cast<float>(value));
    return *this;
  }

  /**
   * @brief Returns the current value.
   *
   * @return The value as a float.
   */
  float Value() { return value_; }

 private:
  float value_; /**< The normalized cyclic value in [0, 2π). */
};

/**
 * @brief Represents Euler angles with cyclic values for yaw, pitch, and roll.
 */
typedef struct {
  CycleValue yaw;
  CycleValue pit;
  CycleValue rol;
} Eulr;

/**
 * @brief Represents a quaternion (w, x, y, z components).
 */
typedef struct {
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion;

/**
 * @brief Represents a 2D vector.
 */
typedef struct {
  float x;
  float y;
} Vector2;

/**
 * @brief Represents a 3D vector.
 */
typedef struct {
  float x;
  float y;
  float z;
} Vector3;

};  // namespace Type
