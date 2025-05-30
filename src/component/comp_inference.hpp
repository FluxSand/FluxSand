#pragma once

#include <onnxruntime_cxx_api.h>

#include <atomic>
#include <chrono>
#include <ctime>
#include <deque>
#include <format>
#include <fstream>
#include <functional>
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

/* Model output categories */
enum class ModelOutput : int8_t {
  UNRECOGNIZED = -1,           /* Unrecognized motion */
  FLIP_OVER = 0,               /* Rotate 180 degrees (flip over) */
  LONG_VIBRATION = 1,          /* Sustained and strong vibration */
  ROTATE_CLOCKWISE = 2,        /* Rotate clockwise */
  ROTATE_COUNTERCLOCKWISE = 3, /* Rotate counterclockwise */
  SHAKE_BACKWARD = 4,          /* Quick backward shake */
  SHAKE_FORWARD = 5,           /* Quick forward shake */
  SHORT_VIBRATION = 6,         /* Short and slight vibration */
  TILT_LEFT = 7,               /* Tilt left and hold */
  TILT_RIGHT = 8,              /* Tilt right and hold */
  STILL = 9                    /* No motion or slow movement */
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
  /**
   * @brief Constructor for the InferenceEngine.
   * @param model_path Path to the ONNX model file.
   * @param update_ratio Ratio for updating the sensor buffer.
   * @param confidence_threshold Minimum probability required to accept a
   * prediction.
   * @param history_size Number of past predictions stored for voting.
   * @param min_consensus_votes Minimum votes required to confirm a
   * prediction.
   */
  explicit InferenceEngine(const std::string& model_path,
                           float update_ratio = 0.1f,
                           float confidence_threshold = 0.6f,
                           size_t history_size = 5,
                           size_t min_consensus_votes = 3)
      : env_(ORT_LOGGING_LEVEL_WARNING, "ONNXModel"),
        session_options_(),
        session_(env_, model_path.c_str(), session_options_),
        allocator_(),
        ready_(0),
        confidence_threshold_(confidence_threshold),
        history_size_(history_size),
        min_consensus_votes_(min_consensus_votes) {
    /* Retrieve input tensor metadata */
    size_t num_input_nodes = session_.GetInputCount();
    std::cout << "Model Input Tensors:\n";

    for (size_t i = 0; i < num_input_nodes; ++i) {
      auto name = session_.GetInputNameAllocated(i, allocator_);
      input_names_.push_back(name.get());
      input_names_cstr_.push_back(input_names_.back().c_str());

      Ort::TypeInfo input_type_info = session_.GetInputTypeInfo(i);
      auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
      input_shape_ = input_tensor_info.GetShape();

      /* Handle dynamic batch dimension */
      if (input_shape_[0] == -1) {
        input_shape_[0] = 1;
      }

      std::cout << "  Name: " << name.get() << "\n  Shape: ["
                << VectorToString(input_shape_) << "]\n";

      input_tensor_size_ =
          std::accumulate(input_shape_.begin(), input_shape_.end(), 1,
                          std::multiplies<int64_t>());
    }

    /* Retrieve output tensor metadata */
    size_t num_output_nodes = session_.GetOutputCount();
    for (size_t i = 0; i < num_output_nodes; ++i) {
      output_names_.push_back(
          session_.GetOutputNameAllocated(i, allocator_).get());
      output_names_cstr_.push_back(output_names_.back().c_str());

      Ort::TypeInfo output_type_info = session_.GetOutputTypeInfo(i);
      auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
      output_shape_ = output_tensor_info.GetShape();

      std::cout << "Model Output Tensor:\n  Name: " << output_names_.back()
                << "\n  Shape: [" << VectorToString(output_shape_) << "]\n";
    }

    /* Configure data collection parameters */
    new_data_number_ =
        static_cast<int>(static_cast<float>(input_shape_[1]) * update_ratio);

    std::cout << std::format("Model initialized: {}\n\n", model_path);

    /* Start inference thread */
    inference_thread_ = std::thread(&InferenceEngine::InferenceTask, this);
  }

  void RecordData(int duration, const char* prefix) {
    /* Generate timestamped filename */
    auto t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    std::string filename =
        std::format("{}_record_{:04}{:02}{:02}_{:02}{:02}{:02}.csv", prefix,
                    tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
                    tm.tm_min, tm.tm_sec);

    /* Create data file */
    std::ofstream file(filename);
    if (!file.is_open()) {
      std::cerr << std::format("Failed to create: {}\n", filename);
      return;
    }

    /* Write CSV header */
    file << "Pitch,Roll,Gyro_X,Gyro_Y,Gyro_Z,Accel_X,Accel_Y,Accel_Z\n";

    /* Collect data at 1kHz */
    constexpr std::chrono::microseconds RUN_CYCLE(1000);
    auto next_sample = std::chrono::steady_clock::now();

    for (int i = 0; i < duration; ++i) {
      file << std::format("{},{},{},{},{},{},{},{}\n", eulr_.pit.Value(),
                          eulr_.rol.Value(), gyro_.x, gyro_.y, gyro_.z,
                          accel_.x, accel_.y, accel_.z);

      next_sample += RUN_CYCLE;
      std::this_thread::sleep_until(next_sample);
    }

    file.close();
    std::cout << std::format("Recorded {} samples to {}\n", duration, filename);
  }

  /* Main inference processing loop */
  void InferenceTask() {
    int update_counter = 0;

    while (true) {
      ready_.acquire();

      /* Update sensor buffer */
      CollectSensorData();

      if (update_counter++ >= new_data_number_) {
        update_counter = 0;

        if (sensor_buffer_.size() >= input_tensor_size_) {
          std::vector<float> input_data(
              sensor_buffer_.begin(),
              sensor_buffer_.begin() + static_cast<int>(input_tensor_size_));
          static ModelOutput last_result = ModelOutput::UNRECOGNIZED;
          ModelOutput result = RunInference(input_data);
          if (last_result != result && result != ModelOutput::UNRECOGNIZED) {
            last_result = result;
            if (data_callback_) {
              data_callback_(result);
            }
          }
        }
      }
    }
  }

  void OnData(const Type::Vector3& accel, const Type::Vector3& gyro,
              const Type::Eulr& eulr) {
    accel_ = accel;
    gyro_ = gyro;
    eulr_ = eulr;
    ready_.release();
  }

  void RegisterDataCallback(const std::function<void(ModelOutput)>& callback) {
    data_callback_ = callback;
  }

  void RunUnitTest() {
    std::cout
        << "[InferenceEngine::UnitTest] Starting inference timing test...\n";

    const int N = 50;  // Number of inference runs
    std::vector<float> dummy_input(input_tensor_size_, 0.0f);  // All zero input

    std::vector<float> timings_ms;
    timings_ms.reserve(N);

    for (int i = 0; i < N; ++i) {
      auto t_start = std::chrono::high_resolution_clock::now();
      ModelOutput result = RunInference(dummy_input);
      auto t_end = std::chrono::high_resolution_clock::now();

      float ms =
          std::chrono::duration<float, std::milli>(t_end - t_start).count();
      timings_ms.push_back(ms);

      std::cout << std::format("Run {:02d} → {:>7.3f} ms | Result: {}\n", i + 1,
                               ms, LABELS.at(result));
    }

    auto [min_it, max_it] =
        std::minmax_element(timings_ms.begin(), timings_ms.end());
    float avg = std::accumulate(timings_ms.begin(), timings_ms.end(), 0.0f) / N;

    std::cout << "\n[Inference Timing Summary]\n";
    std::cout << std::format("  Total Runs    : {}\n", N);
    std::cout << std::format("  Min Time (ms) : {:>7.3f}\n", *min_it);
    std::cout << std::format("  Max Time (ms) : {:>7.3f}\n", *max_it);
    std::cout << std::format("  Avg Time (ms) : {:>7.3f}\n", avg);
    std::cout << "[InferenceEngine::UnitTest] ✅ Timing test complete.\n";
  }

 private:
  /* Sensor data collection */
  void CollectSensorData() {
    /* Normalize and store sensor readings */
    sensor_buffer_.push_back(eulr_.pit.Value());
    sensor_buffer_.push_back(eulr_.rol.Value());
    sensor_buffer_.push_back(gyro_.x);
    sensor_buffer_.push_back(gyro_.y);
    sensor_buffer_.push_back(gyro_.z);
    sensor_buffer_.push_back(accel_.x / GRAVITY);
    sensor_buffer_.push_back(accel_.y / GRAVITY);
    sensor_buffer_.push_back(accel_.z / GRAVITY);

    /* Maintain fixed buffer size */
    while (sensor_buffer_.size() > input_tensor_size_) {
      sensor_buffer_.pop_front();
    }
  }

  /**
   * @brief Runs inference on the collected sensor data.
   * @param input_data Vector containing preprocessed sensor data.
   * @return The predicted motion category as a string label.
   */
  ModelOutput RunInference(std::vector<float>& input_data) {
    /* Validate output tensor dimensions */
    if (output_shape_.size() < 2 || output_shape_[1] <= 0) {
      std::perror("Invalid model output dimensions");
    }

    /* Prepare input tensor */
    Ort::MemoryInfo memory_info =
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, input_data.data(), input_data.size(), input_shape_.data(),
        input_shape_.size());

    /* Perform inference */
    auto outputs =
        session_.Run(Ort::RunOptions{nullptr}, input_names_cstr_.data(),
                     &input_tensor, 1, output_names_cstr_.data(), 1);

    /* Get the class with the highest probability */
    float* probs = outputs.front().GetTensorMutableData<float>();
    auto max_prob = std::max_element(probs, probs + output_shape_[1]);
    int pred_class = static_cast<int>(max_prob - probs);

    /* Apply confidence threshold */
    if (*max_prob < confidence_threshold_) {
      pred_class = static_cast<int>(ModelOutput::UNRECOGNIZED);
    }

    /* Update prediction history */
    prediction_history_.push_back(static_cast<ModelOutput>(pred_class));
    if (prediction_history_.size() > history_size_) {
      prediction_history_.pop_front();
    }

    /* Perform majority voting to ensure stable predictions */
    std::map<ModelOutput, int> votes;
    for (auto label : prediction_history_) {
      votes[label]++;
    }

    auto consensus =
        std::max_element(votes.begin(), votes.end(),
                         [](auto& a, auto& b) { return a.second < b.second; });

    /* Return the final motion category if consensus is reached */
    ModelOutput result = (consensus->second >= min_consensus_votes_)
                             ? consensus->first
                             : ModelOutput::UNRECOGNIZED;

    return result;
  }

  /* Helper to format vector for logging */
  template <typename T>
  std::string VectorToString(const std::vector<T>& vec) {
    std::stringstream ss;
    for (size_t i = 0; i < vec.size(); ++i) {
      ss << vec[i] << (i < vec.size() - 1 ? ", " : "");
    }
    return ss.str();
  }

  /* ONNX runtime components */
  Ort::Env env_;
  Ort::SessionOptions session_options_;
  Ort::Session session_;
  Ort::AllocatorWithDefaultOptions allocator_;

  /* Model interface metadata */
  std::vector<std::string> input_names_;
  std::vector<const char*> input_names_cstr_;
  std::vector<int64_t> input_shape_;
  size_t input_tensor_size_;

  std::vector<std::string> output_names_;
  std::vector<const char*> output_names_cstr_;
  std::vector<int64_t> output_shape_;

  /* Data buffers */
  std::deque<float> sensor_buffer_;
  std::deque<ModelOutput> prediction_history_;

  /* Minimum probability required to accept a prediction */
  float confidence_threshold_;
  /* Number of past predictions stored for voting */
  size_t history_size_;
  /* Minimum votes required to confirm a prediction */
  size_t min_consensus_votes_;

  /* Sensor state */
  Type::Eulr eulr_{};
  Type::Vector3 gyro_{};
  Type::Vector3 accel_{};

  /* Callback function */
  std::function<void(ModelOutput)> data_callback_;

  /* Thread control */
  std::binary_semaphore ready_;
  std::thread inference_thread_;
  int new_data_number_;
};
