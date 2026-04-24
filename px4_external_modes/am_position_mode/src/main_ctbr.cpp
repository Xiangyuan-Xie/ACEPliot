/****************************************************************************
 * Copyright (c) 2026 Xiangyuan Xie <dragonboat_xxy@163.com>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <am_position_ctbr_mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>

static constexpr char kNodeName[] = "am_position_ctbr";
static constexpr bool kEnableDebugOutput = true;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<px4_ros2::NodeWithMode<AmPositionCTBRMode>>(
      kNodeName,
      kEnableDebugOutput));
  rclcpp::shutdown();
  return 0;
}
