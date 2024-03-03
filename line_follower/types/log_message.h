// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_TYPES_LOG_MESSAGE_H_
#define LINE_FOLLOWER_TYPES_LOG_MESSAGE_H_

#include <cstdlib>

#include "line_follower/types/system_time.h"

namespace line_follower {

constexpr std::size_t kMaxMessageLength{100U};
constexpr std::size_t kMaxFileNameLength{100U};

/// A logging message
struct LogMessage final {
    SystemTime timestamp;
    char message[kMaxMessageLength];
    char file[kMaxFileNameLength];
    int line;
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_TYPES_LOG_MESSAGE_H_
