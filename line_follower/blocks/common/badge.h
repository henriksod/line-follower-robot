// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_COMMON_BADGE_H_
#define LINE_FOLLOWER_BLOCKS_COMMON_BADGE_H_

namespace line_follower {
/// @brief A badge that can only be constructed by class of type T.
/// @tparam T The class which is allowed to construct the badge.
template<class T>
class Badge final {
  friend T;
  Badge() {}
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_COMMON_BADGE_H_
