// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_DEPLOYMENT_ARDUINO_PH0_LINE_FOLLOWER_LINEFOLLOWER_H_
#define LINE_FOLLOWER_DEPLOYMENT_ARDUINO_PH0_LINE_FOLLOWER_LINEFOLLOWER_H_

#include <Arduino.h>

#include "line_follower/external/api/encoder_data_agent.h"
#include "line_follower/external/api/ir_sensor_array_data_agent.h"
#include "line_follower/external/api/line_following_agent.h"
#include "line_follower/external/api/logging.h"
#include "line_follower/external/api/logging_agent.h"
#include "line_follower/external/api/motor_signal_agent.h"
#include "line_follower/external/api/scheduler_agent.h"
#include "line_follower/external/api/time_agent.h"
#include "line_follower/external/types/log_message.h"
#include "line_follower/external/types/pin.h"

namespace line_follower {
namespace arduino {

uint32_t GET_SYSTEM_TIME_MICROS() {
    return micros();
}

void PRINT_MESSAGE(LogMessage const& message) {
    Serial.print(message.message);
}

void DIGITAL_WRITE(DigitalPin& pin, PinState const state) {
    if (state == PinState::kHigh) {
        digitalWrite(pin.id, HIGH);
    } else if (state == PinState::kLow) {
        digitalWrite(pin.id, LOW);
    }
    pin.state = state;
}

void ANALOG_WRITE(AnalogPin& pin, uint32_t const value) {
    analogWrite(pin.id, value);
    pin.value = value;
}

PinState DIGITAL_READ(DigitalPin& pin) {
    uint32_t state{digitalRead(pin.id)};
    if (state == HIGH) {
        pin.state = PinState::kHigh;
        return PinState::kHigh;
    } else if (state == LOW) {
        pin.state = PinState::kLow;
        return PinState::kLow;
    }
    pin.state = PinState::kLow;
    return PinState::kLow;
}

uint32_t ANALOG_READ(AnalogPin& pin) {
    uint32_t value{analogRead(pin.id)};
    pin.value = value;
    return value;
}

void SET_PIN_MODE(DigitalPin& pin, PinMode const mode) {
    switch (mode) {
        case PinMode::kInput: pinMode(pin.id, INPUT); break;
        case PinMode::kOutput: pinMode(pin.id, OUTPUT); break;
        case PinMode::kInputPullup: pinMode(pin.id, INPUT_PULLUP); break;
        default: pinMode(pin.id, INPUT); break;
    }
    pin.mode = mode;
}

void SET_PIN_MODE(AnalogPin& pin, PinMode const mode) {
    switch (mode) {
        case PinMode::kInput: pinMode(pin.id, INPUT); break;
        case PinMode::kOutput: pinMode(pin.id, OUTPUT); break;
        case PinMode::kInputPullup: pinMode(pin.id, INPUT_PULLUP); break;
        default: pinMode(pin.id, INPUT); break;
    }
    pin.mode = mode;
}

}  // namespace arduino
}  // namespace line_follower

#endif  // LINE_FOLLOWER_DEPLOYMENT_ARDUINO_PH0_LINE_FOLLOWER_LINEFOLLOWER_H_
