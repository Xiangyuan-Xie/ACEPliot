#include <policy_inference/onnx_policy.hpp>

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace policy_inference
{
namespace
{

std::vector<int64_t> resolveInputShape(
  const std::vector<int64_t> & model_shape,
  std::size_t element_count,
  const std::string & input_name)
{
  std::vector<int64_t> resolved_shape = model_shape;
  std::size_t static_element_count = 1;
  std::size_t dynamic_dimension_count = 0;
  std::size_t dynamic_dimension_index = 0;

  for (std::size_t index = 0; index < resolved_shape.size(); ++index) {
    const int64_t dimension = resolved_shape[index];
    if (dimension < 0) {
      ++dynamic_dimension_count;
      dynamic_dimension_index = index;
      continue;
    }
    if (dimension == 0) {
      static_element_count = 0;
      continue;
    }
    const auto concrete_dimension = static_cast<std::size_t>(dimension);
    if (concrete_dimension >
      std::numeric_limits<std::size_t>::max() / static_element_count)
    {
      throw std::overflow_error("ONNX policy input shape is too large: " + input_name);
    }
    static_element_count *= concrete_dimension;
  }

  if (dynamic_dimension_count > 1) {
    throw std::invalid_argument(
            "ONNX policy input '" + input_name +
            "' has multiple dynamic dimensions; a flat Tensor cannot infer an unambiguous shape");
  }

  if (dynamic_dimension_count == 0) {
    if (static_element_count != element_count) {
      throw std::invalid_argument(
              "ONNX policy input '" + input_name + "' expects " +
              std::to_string(static_element_count) + " elements, received " +
              std::to_string(element_count));
    }
    return resolved_shape;
  }

  if (static_element_count == 0 || element_count == 0 ||
    element_count % static_element_count != 0)
  {
    throw std::invalid_argument(
            "ONNX policy input '" + input_name +
            "' element count cannot resolve its dynamic dimension");
  }

  const std::size_t dynamic_dimension = element_count / static_element_count;
  if (dynamic_dimension > static_cast<std::size_t>(std::numeric_limits<int64_t>::max())) {
    throw std::overflow_error("ONNX policy input shape is too large: " + input_name);
  }
  resolved_shape[dynamic_dimension_index] = static_cast<int64_t>(dynamic_dimension);
  return resolved_shape;
}

}  // namespace

class OnnxPolicy::Impl final
{
public:
  explicit Impl(const std::filesystem::path & model_path)
  : model_path_(model_path),
    environment_(ORT_LOGGING_LEVEL_WARNING, "policy_inference"),
    memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
  {
    if (model_path_.empty()) {
      throw std::invalid_argument("ONNX model path must not be empty");
    }
    if (!std::filesystem::is_regular_file(model_path_)) {
      throw std::invalid_argument("ONNX model does not exist: " + model_path_.string());
    }

    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    try {
      session_ = std::make_unique<Ort::Session>(
        environment_, model_path_.string().c_str(), options);
      readModelContract();
    } catch (const Ort::Exception & error) {
      // Keep the complete ORT message. This includes missing external-data sidecar paths.
      throw std::runtime_error(
              "Failed to load ONNX model '" + model_path_.string() + "': " + error.what());
    }
  }

  TensorMap infer(const TensorMap & inputs) const
  {
    for (const auto & [name, tensor] : inputs) {
      static_cast<void>(tensor);
      if (input_shapes_.find(name) == input_shapes_.end()) {
        throw std::invalid_argument("Unexpected ONNX policy input: " + name);
      }
    }

    std::vector<Ort::Value> input_values;
    input_values.reserve(input_names_.size());
    for (const std::string & name : input_names_) {
      const auto input = inputs.find(name);
      if (input == inputs.end()) {
        throw std::invalid_argument("Missing ONNX policy input: " + name);
      }
      input_values.emplace_back(makeTensor(name, input->second));
    }

    try {
      const auto output_values = session_->Run(
        Ort::RunOptions{nullptr}, input_name_views_.data(), input_values.data(),
        input_values.size(), output_name_views_.data(), output_name_views_.size());

      if (output_values.size() != output_names_.size()) {
        throw std::runtime_error("ONNX Runtime returned an unexpected output count");
      }

      TensorMap outputs;
      outputs.reserve(output_names_.size());
      for (std::size_t index = 0; index < output_names_.size(); ++index) {
        if (!output_values[index].IsTensor()) {
          throw std::runtime_error(
                  "ONNX policy output '" + output_names_[index] + "' is not a tensor");
        }
        const auto tensor_info = output_values[index].GetTensorTypeAndShapeInfo();
        if (tensor_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
          throw std::runtime_error(
                  "ONNX policy output '" + output_names_[index] + "' is not float");
        }
        const std::size_t element_count = tensor_info.GetElementCount();
        Tensor output(element_count);
        if (element_count > 0) {
          const float * data = output_values[index].GetTensorData<float>();
          std::copy_n(data, element_count, output.begin());
        }
        outputs.emplace(output_names_[index], std::move(output));
      }
      return outputs;
    } catch (const Ort::Exception & error) {
      throw std::runtime_error("ONNX inference failed: " + std::string(error.what()));
    }
  }

  bool hasInput(const std::string & name) const noexcept
  {
    return input_shapes_.find(name) != input_shapes_.end();
  }

  bool hasOutput(const std::string & name) const noexcept
  {
    return output_shapes_.find(name) != output_shapes_.end();
  }

  const std::vector<int64_t> & inputShape(const std::string & name) const
  {
    return shapeFor(input_shapes_, name, "input");
  }

  const std::vector<int64_t> & outputShape(const std::string & name) const
  {
    return shapeFor(output_shapes_, name, "output");
  }

  const std::vector<std::string> & inputNames() const noexcept
  {
    return input_names_;
  }

  const std::vector<std::string> & outputNames() const noexcept
  {
    return output_names_;
  }

  const std::filesystem::path & modelPath() const noexcept
  {
    return model_path_;
  }

private:
  using ShapeMap = std::unordered_map<std::string, std::vector<int64_t>>;

  static const std::vector<int64_t> & shapeFor(
    const ShapeMap & shapes,
    const std::string & name,
    const std::string & kind)
  {
    const auto shape = shapes.find(name);
    if (shape == shapes.end()) {
      throw std::out_of_range("ONNX policy " + kind + " does not exist: " + name);
    }
    return shape->second;
  }

  void readModelContract()
  {
    const std::size_t input_count = session_->GetInputCount();
    const std::size_t output_count = session_->GetOutputCount();
    if (input_count == 0 || output_count == 0) {
      throw std::invalid_argument("ONNX policy must expose at least one input and one output");
    }

    input_names_.reserve(input_count);
    output_names_.reserve(output_count);
    Ort::AllocatorWithDefaultOptions allocator;
    for (std::size_t index = 0; index < input_count; ++index) {
      auto name = session_->GetInputNameAllocated(index, allocator);
      if (name.get() == nullptr || name.get()[0] == '\0') {
        throw std::invalid_argument("ONNX policy contains an unnamed input");
      }
      input_names_.emplace_back(name.get());
      const auto type_info = session_->GetInputTypeInfo(index);
      const auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
      if (tensor_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        throw std::invalid_argument(
                "ONNX policy input '" + input_names_.back() + "' must use float tensors");
      }
      if (!input_shapes_.emplace(input_names_.back(), tensor_info.GetShape()).second) {
        throw std::invalid_argument("ONNX policy contains a duplicate input name");
      }
    }

    for (std::size_t index = 0; index < output_count; ++index) {
      auto name = session_->GetOutputNameAllocated(index, allocator);
      if (name.get() == nullptr || name.get()[0] == '\0') {
        throw std::invalid_argument("ONNX policy contains an unnamed output");
      }
      output_names_.emplace_back(name.get());
      const auto type_info = session_->GetOutputTypeInfo(index);
      const auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
      if (tensor_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        throw std::invalid_argument(
                "ONNX policy output '" + output_names_.back() + "' must use float tensors");
      }
      if (!output_shapes_.emplace(output_names_.back(), tensor_info.GetShape()).second) {
        throw std::invalid_argument("ONNX policy contains a duplicate output name");
      }
    }

    std::transform(
      input_names_.begin(), input_names_.end(), std::back_inserter(input_name_views_),
      [](const std::string & name) {return name.c_str();});
    std::transform(
      output_names_.begin(), output_names_.end(), std::back_inserter(output_name_views_),
      [](const std::string & name) {return name.c_str();});
  }

  Ort::Value makeTensor(const std::string & name, const Tensor & tensor) const
  {
    std::vector<int64_t> shape = resolveInputShape(inputShape(name), tensor.size(), name);
    return Ort::Value::CreateTensor<float>(
      memory_info_, const_cast<float *>(tensor.data()), tensor.size(), shape.data(), shape.size());
  }

  std::filesystem::path model_path_;
  Ort::Env environment_;
  Ort::MemoryInfo memory_info_;
  std::unique_ptr<Ort::Session> session_;
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;
  std::vector<const char *> input_name_views_;
  std::vector<const char *> output_name_views_;
  ShapeMap input_shapes_;
  ShapeMap output_shapes_;
};

OnnxPolicy::OnnxPolicy(const std::filesystem::path & model_path)
: impl_(std::make_unique<Impl>(model_path))
{
}

OnnxPolicy::~OnnxPolicy() = default;

TensorMap OnnxPolicy::infer(const TensorMap & inputs) const
{
  return impl_->infer(inputs);
}

bool OnnxPolicy::hasInput(const std::string & name) const noexcept
{
  return impl_->hasInput(name);
}

bool OnnxPolicy::hasOutput(const std::string & name) const noexcept
{
  return impl_->hasOutput(name);
}

const std::vector<int64_t> & OnnxPolicy::inputShape(const std::string & name) const
{
  return impl_->inputShape(name);
}

const std::vector<int64_t> & OnnxPolicy::outputShape(const std::string & name) const
{
  return impl_->outputShape(name);
}

const std::vector<std::string> & OnnxPolicy::inputNames() const noexcept
{
  return impl_->inputNames();
}

const std::vector<std::string> & OnnxPolicy::outputNames() const noexcept
{
  return impl_->outputNames();
}

const std::filesystem::path & OnnxPolicy::modelPath() const noexcept
{
  return impl_->modelPath();
}

}  // namespace policy_inference
