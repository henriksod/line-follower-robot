// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_SERVICE_AGENTS_COMMON_LOGGING_H_
#define LINE_FOLLOWER_SERVICE_AGENTS_COMMON_LOGGING_H_

#include <cstdint>

#include "line_follower/service_agents/logging/logging_agent.h"

namespace line_follower {

// Do not use
#define LOG_(message, verbosity) \
    LoggingAgent::getInstance().dispatchMessage(message, __FILE__, __LINE__, verbosity)

// Use these macros for logging
#define LOG_DEBUG(message) LOG_(message, __FILE__, __LINE__, detail::LoggingVerbosityLevel::kDebug)
#define LOG_INFO(message) LOG_(message, __FILE__, __LINE__, detail::LoggingVerbosityLevel::kInfo)
#define LOG_WARN(message) LOG_(message, __FILE__, __LINE__, detail::LoggingVerbosityLevel::kWarn)
#define LOG_ERROR(message) LOG_(message, __FILE__, __LINE__, detail::LoggingVerbosityLevel::kError)
#define LOG_FATAL_ABORT(message) \
    LOG_(message, __FILE__, __LINE__, detail::LoggingVerbosityLevel::kFatal)

}  // namespace line_follower

#endif  // LINE_FOLLOWER_SERVICE_AGENTS_COMMON_LOGGING_H_
