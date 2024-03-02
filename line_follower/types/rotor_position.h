// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_TYPES_ROTOR_POSITION_H_
#define LINE_FOLLOWER_TYPES_ROTOR_POSITION_H_

#include <cstdint>

namespace line_follower {
/// A rotor position
struct RotorPosition final {
  /// Rotor position in steps from initial configuration
  int64_t step_position;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_TYPES_ROTOR_POSITION_H_
