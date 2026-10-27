// What virtual_gripper computes, apart from ROS: where a finger is after one step toward
// its commanded opening, and whether a command can be taken.
#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>

namespace rby1_additional_tools {

// The opening after `dt` s moving from `now` toward `goal` at `speed` (m/s), without
// passing it.
inline double step_toward(double now, double goal, double speed, double dt) {
  const double step = speed * dt;
  return goal > now ? std::min(goal, now + step) : std::max(goal, now - step);
}

// Why `opening` (m, each finger) cannot be a command for a gripper opening up to
// `max_opening`, or nothing when it can.
inline std::optional<std::string> refused(double opening, double max_opening) {
  if (!std::isfinite(opening)) return "not a number";
  if (opening < 0.0 || opening > max_opening) {
    return "outside 0 (closed) .. " + std::to_string(max_opening) + " m (open)";
  }
  return std::nullopt;
}

}  // namespace rby1_additional_tools
