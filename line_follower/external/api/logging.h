// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_API_LOGGING_H_
#define LINE_FOLLOWER_EXTERNAL_API_LOGGING_H_

#include <cstdint>
#include <string>

#include "line_follower/blocks/common/string.h"
#include "line_follower/external/api/logging_agent.h"

namespace line_follower {

// Do not use
#define LOG_(message, verbosity) \
    LoggingAgent::getInstance().dispatchMessage(message, __FILE__, __LINE__, verbosity)

// Use these macros for logging
#define LOG_DEBUG(format, ...) \
    LOG_(string_format(format, "", ##__VA_ARGS__), detail::LoggingVerbosityLevel::kDebug)
#define LOG_INFO(format, ...) \
    LOG_(string_format(format, "", ##__VA_ARGS__), detail::LoggingVerbosityLevel::kInfo)
#define LOG_WARN(format, ...) \
    LOG_(string_format(format, "", ##__VA_ARGS__), detail::LoggingVerbosityLevel::kWarn)
#define LOG_ERROR(format, ...) \
    LOG_(string_format(format, "", ##__VA_ARGS__), detail::LoggingVerbosityLevel::kError)
#define LOG_FATAL_ABORT(format, ...) \
    LOG_(string_format(format, "", ##__VA_ARGS__), detail::LoggingVerbosityLevel::kFatal)

inline detail::LoggingVerbosityLevel parseVerbosityLevel(std::string const& str) {
    if (str == "debug") return detail::LoggingVerbosityLevel::kDebug;
    if (str == "info") return detail::LoggingVerbosityLevel::kInfo;
    if (str == "warn") return detail::LoggingVerbosityLevel::kWarn;
    if (str == "error") return detail::LoggingVerbosityLevel::kError;
    return detail::LoggingVerbosityLevel::kFatal;
}

}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_API_LOGGING_H_
