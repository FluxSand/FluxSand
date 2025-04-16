#pragma once

/*
  MadgwickAHRS
*/

#include <fstream>
#include <semaphore>

#include "bsp.hpp"
#include "comp_type.hpp"

class AHRS {
 public:
  AHRS() : ready_(0) {
    quat_.q0 = -1.0f;
    quat_.q1 = 0.0f;
    quat_.q2 = 0.0f;
    quat_.q3 = 0.0f;

    start_ = now_ = last_wakeup_ =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch());

    thread_ = std::thread(&AHRS::ThreadTask, this);
  }

  void ThreadTask() {
    while (true) {
      ready_.acquire();
      Update();
      GetEulr();
    }
  }

  void Update() {
    static float recip_norm;
    static float s0, s1, s2, s3;
    static float q_dot1, q_dot2, q_dot3, q_dot4;
    static float q_2q0, q_2q1, q_2q2, q_2q3, q_4q0, q_4q1, q_4q2, q_8q1, q_8q2,
        q0q0, q1q1, q2q2, q3q3;

    now_ = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch());

    dt_ = static_cast<float>(now_.count() - last_wakeup_.count()) / 1000000.0f;
    last_wakeup_ = now_;

    float ax = accel_.x;
    float ay = accel_.y;
    float az = accel_.z;

    float gx = gyro_.x;
    float gy = gyro_.y;
    float gz = gyro_.z;

    /* Rate of change of quaternion from gyroscope */
    q_dot1 = 0.5f * (-quat_.q1 * gx - quat_.q2 * gy - quat_.q3 * gz);
    q_dot2 = 0.5f * (quat_.q0 * gx + quat_.q2 * gz - quat_.q3 * gy);
    q_dot3 = 0.5f * (quat_.q0 * gy - quat_.q1 * gz + quat_.q3 * gx);
    q_dot4 = 0.5f * (quat_.q0 * gz + quat_.q1 * gy - quat_.q2 * gx);

    /* Compute feedback only if accelerometer measurement valid (avoids NaN in
     * accelerometer normalisation) */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
      /* Normalise accelerometer measurement */
      recip_norm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
      ax *= recip_norm;
      ay *= recip_norm;
      az *= recip_norm;

      /* Auxiliary variables to avoid repeated arithmetic */
      q_2q0 = 2.0f * quat_.q0;
      q_2q1 = 2.0f * quat_.q1;
      q_2q2 = 2.0f * quat_.q2;
      q_2q3 = 2.0f * quat_.q3;
      q_4q0 = 4.0f * quat_.q0;
      q_4q1 = 4.0f * quat_.q1;
      q_4q2 = 4.0f * quat_.q2;
      q_8q1 = 8.0f * quat_.q1;
      q_8q2 = 8.0f * quat_.q2;
      q0q0 = quat_.q0 * quat_.q0;
      q1q1 = quat_.q1 * quat_.q1;
      q2q2 = quat_.q2 * quat_.q2;
      q3q3 = quat_.q3 * quat_.q3;

      /* Gradient decent algorithm corrective step */
      s0 = q_4q0 * q2q2 + q_2q2 * ax + q_4q0 * q1q1 - q_2q1 * ay;
      s1 = q_4q1 * q3q3 - q_2q3 * ax + 4.0f * q0q0 * quat_.q1 - q_2q0 * ay -
           q_4q1 + q_8q1 * q1q1 + q_8q1 * q2q2 + q_4q1 * az;
      s2 = 4.0f * q0q0 * quat_.q2 + q_2q0 * ax + q_4q2 * q3q3 - q_2q3 * ay -
           q_4q2 + q_8q2 * q1q1 + q_8q2 * q2q2 + q_4q2 * az;
      s3 = 4.0f * q1q1 * quat_.q3 - q_2q1 * ax + 4.0f * q2q2 * quat_.q3 -
           q_2q2 * ay;

      /* normalise step magnitude */
      recip_norm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);

      s0 *= recip_norm;
      s1 *= recip_norm;
      s2 *= recip_norm;
      s3 *= recip_norm;

      float beta_imu = 0;

      if (now_.count() - start_.count() > 1000000) {
        beta_imu = 0.07f;
      } else {
        beta_imu = 10.0f;
      }

      /* Apply feedback step */
      q_dot1 -= beta_imu * s0;
      q_dot2 -= beta_imu * s1;
      q_dot3 -= beta_imu * s2;
      q_dot4 -= beta_imu * s3;
    }

    /* Integrate rate of change of quaternion to yield quaternion */
    quat_.q0 += q_dot1 * dt_;
    quat_.q1 += q_dot2 * dt_;
    quat_.q2 += q_dot3 * dt_;
    quat_.q3 += q_dot4 * dt_;

    /* Normalise quaternion */
    recip_norm = 1.0f / sqrtf(quat_.q0 * quat_.q0 + quat_.q1 * quat_.q1 +
                              quat_.q2 * quat_.q2 + quat_.q3 * quat_.q3);
    quat_.q0 *= recip_norm;
    quat_.q1 *= recip_norm;
    quat_.q2 *= recip_norm;
    quat_.q3 *= recip_norm;
  }

  void GetEulr() {}

  void DisplayData() {
    std::cout << std::format(
        "Quaternion: [q0={:+.4f}, q1={:+.4f}, q2={:+.4f}, q3={:+.4f}] | "
        "Eulr: [rol={:+.4f}, pit={:+.4f}, yaw={:+.4f}] ",
        quat_.q0, quat_.q1, quat_.q2, quat_.q3, eulr_.rol.Value(),
        eulr_.pit.Value(), eulr_.yaw.Value());

    std::cout << std::format(
        "Accel: [x={:+.4f}, y={:+.4f}, z={:+.4f}]] dt={:+.8f}\n",
        filtered_accel_.x, filtered_accel_.y, filtered_accel_.z, dt_);
  }

  void DisplyDataWithoutYaw() {
    std::cout << std::format(
        "Quaternion: [q0={:+.4f}, q1={:+.4f}, q2={:+.4f}, q3={:+.4f}] | "
        "Eulr: [rol={:+.4f}, pit={:+.4f}, yaw={:+.4f}]\n",
        quat_without_z_.q0, quat_without_z_.q1, quat_without_z_.q2,
        quat_without_z_.q3, eulr_without_yaw_.rol.Value(),
        eulr_without_yaw_.pit.Value(), eulr_without_yaw_.yaw.Value());
  }

  void StartRecordData() {
    record_thread_ = std::thread(&AHRS::RecordTask, this);
  }

  void RecordTask() {
    std::ofstream csv_file("imu_data.csv");
    std::chrono::microseconds period_us(static_cast<int>(1000));

    std::chrono::steady_clock::time_point next_time, start_time;
    next_time = start_time = std::chrono::steady_clock::now();
    csv_file << "timestamp,ax,ay,az,gx,gy,gz,q0,q1,q2,q3,rol,pit,yaw,label\n";
    while (true) {
      csv_file << std::to_string((next_time.time_since_epoch().count() -
                                  start_time.time_since_epoch().count()) /
                                 1000)
               << "," << accel_.x << "," << accel_.y << "," << accel_.z << ","
               << gyro_.x << "," << gyro_.y << "," << gyro_.z << "," << quat_.q0
               << "," << quat_.q1 << "," << quat_.q2 << "," << quat_.q3 << ","
               << eulr_.rol.Value() << "," << eulr_.pit.Value() << ","
               << eulr_.yaw.Value() << "," << "1\n";
      csv_file.flush();

      next_time += period_us;
      std::this_thread::sleep_until(next_time);
    }
  }

 private:
  std::chrono::duration<uint64_t, std::ratio<1, 1000000>> last_wakeup_;
  std::chrono::duration<uint64_t, std::ratio<1, 1000000>> now_;
  std::chrono::duration<uint64_t, std::ratio<1, 1000000>> start_;
  float dt_ = 0.0f;

  Type::Quaternion quat_{};
  Type::Eulr eulr_{};
  Type::Quaternion quat_without_z_{};
  Type::Eulr eulr_without_yaw_{};

  Type::Vector3 accel_{};
  Type::Vector3 gyro_{};
  Type::Vector3 filtered_accel_{};

  std::binary_semaphore ready_;

  std::thread thread_, record_thread_; /* Thread */
};
