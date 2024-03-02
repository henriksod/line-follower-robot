// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_TYPES_UNIQUE_ID_H_
#define LINE_FOLLOWER_TYPES_UNIQUE_ID_H_

#include <cstdint>

namespace line_follower {
/// A unique identifier
struct UniqueID final {
  /// Identifier value
  uint32_t id;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_TYPES_UNIQUE_ID_H_
