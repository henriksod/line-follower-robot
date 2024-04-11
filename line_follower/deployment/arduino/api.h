// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_DEPLOYMENT_ARDUINO_API_H_
#define LINE_FOLLOWER_DEPLOYMENT_ARDUINO_API_H_

#include <cstdint>
#include <cstdlib>

namespace line_follower {
namespace arduino {

/// Get the system time in microseconds
extern uint32_t GET_SYSTEM_TIME_MICROS();

/// Print a message to serial output
extern void PRINT_MESSAGE(char* message, size_t length);

}  // namespace arduino
}  // namespace line_follower

#endif  // LINE_FOLLOWER_DEPLOYMENT_ARDUINO_API_H_
