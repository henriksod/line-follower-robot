// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_COMMON_MAYBE_H_
#define LINE_FOLLOWER_BLOCKS_COMMON_MAYBE_H_

#include "line_follower/blocks/common/function.h"

namespace line_follower {
template <typename T>
struct Maybe {
    T value_;
    bool is_nothing_;

    operator bool() const { return !is_nothing_; }
    bool has_value() const { return !is_nothing_; }
    T& value() { return value_; }
};

template <typename T>
Maybe<T> Just(T just) {
    return Maybe<T>{just, false};
}

template <typename T>
Maybe<T> Nothing() {
    return Maybe<T>{T{}, true};
}
}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_COMMON_MAYBE_H_
