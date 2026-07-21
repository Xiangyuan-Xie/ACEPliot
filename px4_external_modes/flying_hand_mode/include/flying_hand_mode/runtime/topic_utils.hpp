#pragma once

#include <string>
#include <string_view>

namespace flying_hand_mode::runtime
{

inline std::string joinTopicPrefix(
  std::string_view prefix, std::string_view topic)
{
  if (prefix.empty()) {
    return std::string(topic);
  }

  std::string result(prefix);
  while (result.size() > 1 && result.back() == '/') {
    result.pop_back();
  }

  std::size_t topic_start = 0;
  while (topic_start < topic.size() && topic[topic_start] == '/') {
    ++topic_start;
  }
  if (result.back() != '/') {
    result.push_back('/');
  }
  result.append(topic.substr(topic_start));
  return result;
}

}  // namespace flying_hand_mode::runtime
