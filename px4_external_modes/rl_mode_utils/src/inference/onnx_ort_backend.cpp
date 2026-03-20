#include "inference/onnx_ort_backend.hpp"
#include <numeric>
#include <sstream>
#include <stdexcept>

OnnxOrtBackend::OnnxOrtBackend(rclcpp::Node & node)
: node_(node), env_(ORT_LOGGING_LEVEL_WARNING, "rl_onnx"),
  mem_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
{
  // Read model path and initialize ORT session options.
  node_.declare_parameter("model_path", "policy.onnx");
  model_path_ = node_.get_parameter("model_path").as_string();

  Ort::SessionOptions opt;
  // Optional: Enable TensorRT / CUDA Execution Providers if needed
  // opt.AppendExecutionProvider_TensorRT({});
  // opt.AppendExecutionProvider_CUDA({});
  opt.SetIntraOpNumThreads(1);   // A typical setting for real-time applications
  session_ = std::make_unique<Ort::Session>(env_, model_path_.c_str(), opt);

  num_inputs_ = session_->GetInputCount();
  num_outputs_ = session_->GetOutputCount();

  Ort::AllocatorWithDefaultOptions allocator;

  // Cache input names and shapes for reuse during runtime inference.
  input_names_.reserve(num_inputs_);
  input_names_cstr_.reserve(num_inputs_);
  for (size_t i = 0; i < num_inputs_; ++i) {
#if defined(USE_ORT_ALLOCATED_NAME_API)
    auto name_ptr = session_->GetInputNameAllocated(i, allocator);
    input_names_.emplace_back(name_ptr.get());
#else
    char * in_name = session_->GetInputName(i, allocator);
    input_names_.emplace_back(in_name);
    allocator.Free(in_name);
#endif
    input_names_cstr_.push_back(input_names_.back().c_str());

    auto type_info = session_->GetInputTypeInfo(i);
    auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
    input_shapes_[input_names_.back()] = tensor_info.GetShape();
  }

  // Cache output names to avoid repeated metadata queries.
  output_names_.reserve(num_outputs_);
  output_names_cstr_.reserve(num_outputs_);
  for (size_t i = 0; i < num_outputs_; ++i) {
#if defined(USE_ORT_ALLOCATED_NAME_API)
    auto name_ptr = session_->GetOutputNameAllocated(i, allocator);
    output_names_.emplace_back(name_ptr.get());
#else
    char * out_name = session_->GetOutputName(i, allocator);
    output_names_.emplace_back(out_name);
    allocator.Free(out_name);
#endif
    output_names_cstr_.push_back(output_names_.back().c_str());
  }

  RCLCPP_INFO(
    node_.get_logger(), "[OnnxOrtBackend] loaded '%s' with %zu inputs and %zu outputs.",
    model_path_.c_str(), num_inputs_, num_outputs_);
}

TensorMap OnnxOrtBackend::forward(const TensorMap & inputs, float /*dt_s*/)
{
  // Build ORT input tensors in the model-defined input order.
  std::vector<Ort::Value> in_vals;
  in_vals.reserve(num_inputs_);

  // Handle single-input models as a special case for convenience
  if (num_inputs_ == 1) {
    if (inputs.empty()) {
      throw std::runtime_error("Single input model expects one input tensor, but got none.");
    }
    if (inputs.size() > 1) {
      RCLCPP_WARN_ONCE(
        node_.get_logger(),
        "Single input model received %zu inputs. Using the first one.", inputs.size());
    }
    const auto & generic_tensor = inputs.begin()->second;
    in_vals.push_back(create_ort_value(generic_tensor, input_names_[0]));
  } else {
    // Multi-input models must match tensors by name, with explicit missing-input errors.
    for (const auto & name : input_names_) {
      const auto it = inputs.find(name);
      if (it == inputs.end()) {
        throw std::runtime_error("Missing required input tensor: '" + name + "'.");
      }
      const auto & generic_tensor = it->second;
      in_vals.push_back(create_ort_value(generic_tensor, name));
    }
  }

  // Run model inference.
  auto out_ort_vals = session_->Run(
    Ort::RunOptions{nullptr},
    input_names_cstr_.data(), in_vals.data(), in_vals.size(),
    output_names_cstr_.data(), num_outputs_);

  // Package outputs back into TensorMap to keep the upstream interface unchanged.
  TensorMap outputs;
  outputs.reserve(num_outputs_);
  for (size_t i = 0; i < num_outputs_; ++i) {
    const float * p = out_ort_vals[i].GetTensorMutableData<float>();
    auto shape_info = out_ort_vals[i].GetTensorTypeAndShapeInfo();
    const auto num_elements = shape_info.GetElementCount();

    // Wrap the raw output data into a std::vector<float> inside the variant
    outputs[output_names_[i]] = std::vector<float>(p, p + num_elements);
  }
  return outputs;
}

bool OnnxOrtBackend::hasInput(const std::string & name) const
{
  return input_shapes_.find(name) != input_shapes_.end();
}

std::vector<int64_t> OnnxOrtBackend::inputShape(const std::string & name) const
{
  auto it = input_shapes_.find(name);
  if (it == input_shapes_.end()) {
    throw std::runtime_error("Input '" + name + "' not found in model.");
  }
  return it->second;
}

Ort::Value OnnxOrtBackend::create_ort_value(const GenericTensor & tensor, const std::string & name)
{
  // Use std::visit for variant dispatch instead of manual type branching.
  return std::visit(
    [&](const auto & data) -> Ort::Value {
      // 1) Validate input dimensions.
      check_input_dimensions(data, name);

      // 2) Create an ORT tensor view according to the concrete input type.
      using T = std::decay_t<decltype(data)>;
      if constexpr (std::is_same_v<T, std::vector<float>>) {
        auto shape = input_shapes_.at(name);
        // Default dynamic batch dimension to batch size 1.
        if (!shape.empty() && shape[0] < 0) {
          shape[0] = 1;
        }
        return Ort::Value::CreateTensor<float>(
          mem_info_, const_cast<float *>(data.data()), data.size(), shape.data(), shape.size());
      } else if constexpr (std::is_same_v<T, cv::Mat>) {
        if (data.type() != CV_32FC1 && data.type() != CV_32FC3) {
          throw std::runtime_error(
            "Input '" + name +
            "' is a cv::Mat but its type is not CV_32FC1 or CV_32FC3.");
        }
        auto shape = input_shapes_.at(name);
        // This path assumes layout already matches the model and the storage is contiguous.
        if (!data.isContinuous()) {
          throw std::runtime_error("Input cv::Mat '" + name + "' is not continuous.");
        }
        return Ort::Value::CreateTensor<float>(
          mem_info_, (float *)data.data, data.total() * data.channels(), shape.data(),
          shape.size());
      }
    },
    tensor
  );
}

void OnnxOrtBackend::check_input_dimensions(
  const std::vector<float> & vec,
  const std::string & name) const
{
  const auto & expected_shape = input_shapes_.at(name);
  size_t expected_elements = 1;
  bool has_dynamic_dim = false;
  for (int64_t dim : expected_shape) {
    if (dim > 0) {
      expected_elements *= dim;
    } else {
      has_dynamic_dim = true;
    }
  }

  if (!has_dynamic_dim && vec.size() != expected_elements) {
    throw std::runtime_error(
            "Input '" + name + "' dimension mismatch. Expected " +
            std::to_string(expected_elements) + " elements, but got " +
            std::to_string(vec.size()));
  }
  if (has_dynamic_dim) {
    // Dynamic-dimension models are not strictly checked by element count here.
  }
}

void OnnxOrtBackend::check_input_dimensions(const cv::Mat & mat, const std::string & name) const
{
  const auto & expected_shape = input_shapes_.at(name);  // e.g., [1, 3, 224, 224]
  const size_t mat_elements = mat.total() * mat.channels();

  size_t expected_elements = 1;
  bool has_dynamic_dim = false;
  for (int64_t dim : expected_shape) {
    if (dim > 0) {
      expected_elements *= dim;
    } else {
      has_dynamic_dim = true;
    }
  }

  if (!has_dynamic_dim && mat_elements != expected_elements) {
    throw std::runtime_error(
            "Input '" + name + "' (cv::Mat) dimension mismatch. Expected " +
            std::to_string(expected_elements) + " elements, but got " +
            std::to_string(mat_elements));
  }

  if (has_dynamic_dim) {
    // Dynamic-dimension models are not strictly checked by element count here.
  }
}
