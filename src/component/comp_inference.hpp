#pragma once

#include <onnxruntime_cxx_api.h>

#include <atomic>
#include <chrono>
#include <ctime>
#include <deque>
#include <format>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <semaphore>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "comp_type.hpp"
#include "message.hpp"
#include "ramfs.hpp"

/* Model output categories */
enum class ModelOutput : int8_t {
  UNRECOGNIZED = -1,            // Unrecognized motion
  FLIP_OVER = 0,                // Quick forward shake
  LONG_VIBRATION = 1,           // Quick backward shake
  ROTATE_CLOCKWISE = 2,         // Tilt left and hold
  ROTATE_COUNTERCLOCKWISE = 3,  // Tilt right and hold
  SHAKE_BACKWARD = 4,           // Rotate 180 degrees (flip over)
  SHAKE_FORWARD = 5,            // Rotate clockwise
  SHORT_VIBRATION = 6,          // Rotate counterclockwise
  TILT_LEFT = 7,                // Short and slight vibration
  TILT_RIGHT = 8,               // Sustained and strong vibration
  STILL = 9                     // No motion or slow, insignificant movement
};
/* Mapping model output to string labels */
static const std::map<ModelOutput, std::string> LABELS = {
    {ModelOutput::UNRECOGNIZED, "Unrecognized"},
    {ModelOutput::SHAKE_FORWARD, "Shake Forward"},
    {ModelOutput::SHAKE_BACKWARD, "Shake Backward"},
    {ModelOutput::TILT_LEFT, "Tilt Left"},
    {ModelOutput::TILT_RIGHT, "Tilt Right"},
    {ModelOutput::FLIP_OVER, "Flip Over"},
    {ModelOutput::ROTATE_CLOCKWISE, "Rotate Clockwise"},
    {ModelOutput::ROTATE_COUNTERCLOCKWISE, "Rotate Counterclockwise"},
    {ModelOutput::SHORT_VIBRATION, "Short Vibration"},
    {ModelOutput::LONG_VIBRATION, "Long Vibration"},
    {ModelOutput::STILL, "Still"}};

class InferenceEngine {
 public:
  explicit InferenceEngine(const std::string& model_path,
                           float update_ratio = 0.1f)
      : env_(ORT_LOGGING_LEVEL_WARNING, "ONNXModel"),
        session_options_(),
        session_(env_, model_path.c_str(), session_options_),
        allocator_(),
        ready_(0) {
    /* Get input topics */
    gravity_free_accel_tp_ = LibXR::Topic(LibXR::Topic::Find("gravity_free_accel"));
    eulr_without_yaw_tp_ = LibXR::Topic(LibXR::Topic::Find("eulr_without_yaw"));
    gyro_tp_ = LibXR::Topic(LibXR::Topic::Find("gyro"));

    /* Retrieve input tensor information */
    size_t num_input_nodes = session_.GetInputCount();
    std::cout << "Model Input Tensors:\n";

    for (size_t i = 0; i < num_input_nodes; ++i) {
      auto name = session_.GetInputNameAllocated(i, allocator_);
      input_names_.push_back(name.get());
      input_names_cstr_.push_back(input_names_.back().c_str());

      Ort::TypeInfo input_type_info = session_.GetInputTypeInfo(i);
      auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
      std::vector<int64_t> input_shape = input_tensor_info.GetShape();

      /* Handle dynamic batch dimension */
      if (input_shape[0] == -1) {
        input_shape[0] = 1;
      }

      std::cout << "  Name: " << name.get() << "\n  Shape: ["
                << VectorToString(input_shape) << "]\n";

      input_shape_ = input_shape;
      input_tensor_size_ =
          std::accumulate(input_shape_.begin(), input_shape_.end(), 1,
                          std::multiplies<int64_t>());
    }

    size_t num_output_nodes = session_.GetOutputCount();
    auto output_name = session_.GetOutputNameAllocated(0, allocator_);

    /* Retrieve output tensor information */
    for (size_t i = 0; i < num_output_nodes; ++i) {
      output_names_.push_back(
          session_.GetOutputNameAllocated(i, allocator_).get());
      output_names_cstr_.push_back(output_names_.back().c_str());

      Ort::TypeInfo output_type_info = session_.GetOutputTypeInfo(0);
      auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
      output_shape_ = output_tensor_info.GetShape();

      std::cout << "Model Output Tensor:\n  Name: " << output_name.get()
                << "\n  Shape: [" << VectorToString(output_shape_) << "]\n";
    }

    /* Calculate update frequency */
    new_data_number_ =
        static_cast<int>(static_cast<float>(input_shape_[1]) * update_ratio);

    std::cout << std::format("Model initialized: {}\n\n", model_path);

    /* Start the inference thread */
    inference_thread_ = std::thread(&InferenceEngine::InferenceTask, this);

    void (*eulr_ready_cb_fun)(bool, InferenceEngine*, LibXR::RawData&) =
        [](bool, InferenceEngine* self, LibXR::RawData& data) {
          memcpy(&self->eulr_without_yaw_, data.addr_, data.size_);
          self->ready_.release();
        };

    void (*gyro_ready_cb_fun)(bool, InferenceEngine*, LibXR::RawData&) =
        [](bool, InferenceEngine* self, LibXR::RawData& data) {
          memcpy(&self->gyro_, data.addr_, data.size_);
        };

    void (*acc_ready_cb_fun)(bool, InferenceEngine*, LibXR::RawData&) =
        [](bool, InferenceEngine* self, LibXR::RawData& data) {
          memcpy(&self->filtered_accel_, data.addr_, data.size_);
        };

    auto accel_cb =
        LibXR::Callback<LibXR::RawData&>::Create(acc_ready_cb_fun, this);
    auto eulr_cb =
        LibXR::Callback<LibXR::RawData&>::Create(eulr_ready_cb_fun, this);
    auto gyro_cb =
        LibXR::Callback<LibXR::RawData&>::Create(gyro_ready_cb_fun, this);

    gravity_free_accel_tp_.RegisterCallback(accel_cb);
    gyro_tp_.RegisterCallback(gyro_cb);
    eulr_without_yaw_tp_.RegisterCallback(eulr_cb);

    cmd_file_ = LibXR::RamFS::CreateFile<InferenceEngine*>(
        "inference_engine",
        [](InferenceEngine*& inference_engine, int argc, char** argv) {
          if (strcmp(argv[1], "record") == 0 && argc == 4) {
            int length = atoi(argv[2]);
            inference_engine->RecordData(length, argv[3]);
          } else {
            std::cout
                << "Usage: inference_engine record <length> <file_prefix>\n";
          }
          return 0;
        },
        this);
  }

  void RecordData(int length, char* prefix) {
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    std::string filename =
        std::format("{}_record_{:04}{:02}{:02}_{:02}{:02}{:02}.csv", prefix,
                    tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
                    tm.tm_min, tm.tm_sec);

    std::ofstream file(filename);
    if (!file.is_open()) {
      std::cerr << std::format("Error opening file: {}\n", filename);
      return;
    }

    file << "Pitch,Roll,Gyro_X,Gyro_Y,Gyro_Z,Accel_X,Accel_Y,"
            "Accel_Z\n";

    std::chrono::microseconds period_us(static_cast<int>(1000));

    std::chrono::steady_clock::time_point next_time =
        std::chrono::steady_clock::now();

    for (size_t i = 0; i < length; ++i) {
      file << std::format("{},{},", eulr_without_yaw_.pit.Value(),
                          eulr_without_yaw_.rol.Value());

      file << std::format("{},{},{},", gyro_.x, gyro_.y, gyro_.z);

      file << std::format("{},{},{}\n", filtered_accel_.x, filtered_accel_.y,
                          filtered_accel_.z);

      next_time += period_us;
      std::this_thread::sleep_until(next_time);
    }

    file.close();
    std::cout << std::format("Data successfully recorded to {}\n", filename);
  }

  /* Inference task loop */
  void InferenceTask() {
    int update_counter = 0;

    while (true) {
      ready_.acquire();

      /* Collect new sensor data periodically */
      CollectSensorData();
      if (update_counter == new_data_number_) {
        update_counter = 0;

        /* Perform inference when the buffer has sufficient data */
        if (sensor_buffer_.size() == input_tensor_size_) {
          std::vector<float> input_data(sensor_buffer_.begin(),
                                        sensor_buffer_.end());
          std::string result = RunInference(input_data);
          std::cout << std::format("Inference Result: {}\n", result);
        }
      }

      update_counter++;
    }
  }

  /* Collect sensor data from MPU9250 */
  void CollectSensorData() {
    sensor_buffer_.push_back(eulr_without_yaw_.pit.Value());
    sensor_buffer_.push_back(eulr_without_yaw_.rol.Value());
    sensor_buffer_.push_back(gyro_.x);
    sensor_buffer_.push_back(gyro_.y);
    sensor_buffer_.push_back(gyro_.z);
    sensor_buffer_.push_back(filtered_accel_.x / GRAVITY);
    sensor_buffer_.push_back(filtered_accel_.y / GRAVITY);
    sensor_buffer_.push_back(filtered_accel_.z / GRAVITY);

    /* Ensure the buffer does not exceed the required tensor size */
    while (sensor_buffer_.size() > input_tensor_size_) {
      sensor_buffer_.pop_front();
    }
  }

  /* Perform ONNX inference */
  std::string RunInference(std::vector<float>& input_data) {
    Ort::MemoryInfo memory_info =
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, input_data.data(), input_data.size(), input_shape_.data(),
        input_shape_.size());

    auto output_tensors =
        session_.Run(Ort::RunOptions{nullptr}, input_names_cstr_.data(),
                     &input_tensor, 1, output_names_cstr_.data(), 1);

    float* output_data = output_tensors.front().GetTensorMutableData<float>();

    int predicted_label = static_cast<int>(
        std::max_element(output_data, output_data + output_shape_[1]) -
        output_data);

    return LABELS.count(static_cast<ModelOutput>(predicted_label))
               ? LABELS.at(static_cast<ModelOutput>(predicted_label))
               : "Unknown";
  }

  /* Helper function to format vector as a string */
  template <typename T>
  std::string VectorToString(const std::vector<T>& vec) {
    std::ostringstream oss;
    for (size_t i = 0; i < vec.size(); ++i) {
      oss << vec[i] << (i < vec.size() - 1 ? ", " : "");
    }
    return oss.str();
  }

  LibXR::RamFS::File& GetFile() { return cmd_file_; }

 private:
  /* ONNX runtime objects */
  Ort::Env env_;
  Ort::SessionOptions session_options_;
  Ort::Session session_;
  Ort::AllocatorWithDefaultOptions allocator_;

  /* Sensor topics */
  LibXR::Topic gravity_free_accel_tp_;
  LibXR::Topic eulr_without_yaw_tp_;
  LibXR::Topic gyro_tp_;

  /* Input tensor metadata */
  std::vector<std::string> input_names_;
  std::vector<const char*> input_names_cstr_;
  std::vector<int64_t> input_shape_;
  size_t input_tensor_size_;

  /* Output tensor metadata */
  std::vector<std::string> output_names_;
  std::vector<const char*> output_names_cstr_;
  std::vector<int64_t> output_shape_;

  /* Sensor data buffer */
  std::deque<float> sensor_buffer_;

  /* Sensor data structures */
  Type::Eulr eulr_without_yaw_{};
  Type::Vector3 gyro_{};
  Type::Vector3 filtered_accel_{};

  /* Determines how frequently new data is added */
  int new_data_number_;

  std::binary_semaphore ready_;

  LibXR::RamFS::File cmd_file_;

  /* Background thread for inference */
  std::thread inference_thread_;
};
