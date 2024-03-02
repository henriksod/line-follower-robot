// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_TYPES_SYSTEM_TIME_H_
#define LINE_FOLLOWER_TYPES_SYSTEM_TIME_H_

#include <cstdint>

namespace line_follower {
/// The time since system start, in microseconds
struct SystemTime final {
  /// Time since start of system, in microseconds
  uint64_t system_time_us;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_TYPES_SYSTEM_TIME_H_
