// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_COMMON_MAYBE_H_
#define LINE_FOLLOWER_BLOCKS_COMMON_MAYBE_H_

#include "line_follower/blocks/common/function.h"

namespace line_follower {
template<typename T>
struct Maybe {
  T    value;
  bool isNothing;

  operator bool() const {
    return !isNothing;
  }
};

template<typename T>
Maybe<T>Just(T just) {
  return Maybe<T>{ just, false };
}

template<typename T>
Maybe<T>Nothing() {
  return Maybe<T>{ T{}, true };
}
}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_COMMON_MAYBE_H_
