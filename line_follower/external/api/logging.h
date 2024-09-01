// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_API_LOGGING_H_
#define LINE_FOLLOWER_EXTERNAL_API_LOGGING_H_

#include <cstdint>
#include <string>

#include "line_follower/blocks/common/string.h"
#include "line_follower/external/api/logging_agent.h"
#include "line_follower/external/types/logging_verbosity_level.h"

namespace line_follower {

// Do not use
#define LOG_(message, verbosity) \
    LoggingAgent::getInstance().dispatchMessage(message, __FILE__, __LINE__, verbosity)

// Use these macros for logging
#define LOG_DEBUG(format, ...) \
    LOG_(string_format(format, "", ##__VA_ARGS__), LoggingVerbosityLevel::kDebug)
#define LOG_INFO(format, ...) \
    LOG_(string_format(format, "", ##__VA_ARGS__), LoggingVerbosityLevel::kInfo)
#define LOG_WARN(format, ...) \
    LOG_(string_format(format, "", ##__VA_ARGS__), LoggingVerbosityLevel::kWarn)
#define LOG_ERROR(format, ...) \
    LOG_(string_format(format, "", ##__VA_ARGS__), LoggingVerbosityLevel::kError)
#define LOG_FATAL_ABORT(format, ...) \
    LOG_(string_format(format, "", ##__VA_ARGS__), LoggingVerbosityLevel::kFatal)

inline LoggingVerbosityLevel parseVerbosityLevel(std::string const& str) {
    if (str == "debug") return LoggingVerbosityLevel::kDebug;
    if (str == "info") return LoggingVerbosityLevel::kInfo;
    if (str == "warn") return LoggingVerbosityLevel::kWarn;
    if (str == "error") return LoggingVerbosityLevel::kError;
    return LoggingVerbosityLevel::kFatal;
}

}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_API_LOGGING_H_
