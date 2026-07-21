#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace policy_inference
{

using Tensor = std::vector<float>;
using TensorMap = std::unordered_map<std::string, Tensor>;

}  // namespace policy_inference
