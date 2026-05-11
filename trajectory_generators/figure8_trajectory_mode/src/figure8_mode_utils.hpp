#pragma once

#include <figure8_trajectory_mode/generator.hpp>

// Keep mode-specific trimming local to this package so the shared generator
// stays focused on producing the full figure-8 trajectory semantics.
inline TrajectorySample makePositionModeSample(const TrajectorySample & input)
{
  TrajectorySample output = input;
  output.velocity.reset();
  output.acceleration.reset();
  output.yaw_rate.reset();
  return output;
}

inline TrajectorySample makeVelocityModeSample(const TrajectorySample & input)
{
  TrajectorySample output = input;
  output.position.reset();
  output.acceleration.reset();
  output.yaw.reset();
  return output;
}
