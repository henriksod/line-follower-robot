// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_DEPLOYMENT_ARDUINO_API_H_
#define LINE_FOLLOWER_DEPLOYMENT_ARDUINO_API_H_

#include <cstdint>
#include <cstdlib>

#include "line_follower/external/types/log_message.h"
#include "line_follower/external/types/pin.h"

namespace line_follower {
namespace arduino {

/// Get the system time in microseconds
extern uint32_t GET_SYSTEM_TIME_MICROS();

/// Print a message to serial output
extern void PRINT_MESSAGE(LogMessage const& message);

/// Write an output state to a pin
extern void DIGITAL_WRITE(DigitalPin& pin, PinState const state);

/// Write an output state to a pin
extern void ANALOG_WRITE(AnalogPin& pin, uint32_t const value);

/// Read an output state to a pin
extern PinState DIGITAL_READ(DigitalPin& pin);

/// Read an output state to a pin
extern uint32_t ANALOG_READ(AnalogPin& pin);

/// Set mode of a digital pin
extern void SET_PIN_MODE(DigitalPin& pin, PinMode const mode);

/// Set mode of an analog pin
extern void SET_PIN_MODE(AnalogPin& pin, PinMode const mode);

}  // namespace arduino
}  // namespace line_follower

#endif  // LINE_FOLLOWER_DEPLOYMENT_ARDUINO_API_H_
