// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_TYPES_LOGGING_VERBOSITY_LEVEL_H_
#define LINE_FOLLOWER_EXTERNAL_TYPES_LOGGING_VERBOSITY_LEVEL_H_

#include <cstdint>

namespace line_follower {

/// Verbosity level for logging
enum class LoggingVerbosityLevel : std::uint8_t { kFatal = 0U, kError, kWarn, kInfo, kDebug };

}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_TYPES_LOGGING_VERBOSITY_LEVEL_H_
