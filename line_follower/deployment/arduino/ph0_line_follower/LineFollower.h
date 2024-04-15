// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_DEPLOYMENT_ARDUINO_PH0_LINE_FOLLOWER_LINEFOLLOWER_H_
#define LINE_FOLLOWER_DEPLOYMENT_ARDUINO_PH0_LINE_FOLLOWER_LINEFOLLOWER_H_

#include <Arduino.h>
#include <QTRSensors.h>

#include "line_follower/external/api/encoder_data_agent.h"
#include "line_follower/external/api/ir_sensor_array_data_agent.h"
#include "line_follower/external/api/line_following_agent.h"
#include "line_follower/external/api/logging.h"
#include "line_follower/external/api/logging_agent.h"
#include "line_follower/external/api/motor_signal_agent.h"
#include "line_follower/external/api/scheduler_agent.h"
#include "line_follower/external/api/time_agent.h"
#include "line_follower/external/types/ir_sensor_array_characteristics.h"
#include "line_follower/external/types/ir_sensor_array_data.h"
#include "line_follower/external/types/ir_sensor_array_pin_configuration.h"
#include "line_follower/external/types/log_message.h"
#include "line_follower/external/types/pin.h"

QTRSensors qtr;

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
    uint32_t value{static_cast<uint32_t>(analogRead(pin.id))};
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

void ATTACH_INTERRUPT(DigitalPin const& pin, PinEventTrigger const trigger, void (*func)(void)) {
    switch (trigger) {
        case PinEventTrigger::kChange: attachInterrupt(pin.id, func, CHANGE); break;
        case PinEventTrigger::kRising: attachInterrupt(pin.id, func, RISING); break;
        case PinEventTrigger::kFalling: attachInterrupt(pin.id, func, FALLING); break;
        default: break;
    }
}

namespace interrupts {

namespace detail {
volatile int32_t last_encoded_left = 0;
volatile int64_t encoder_value_left = 0;
volatile int32_t last_encoded_right = 0;
volatile int64_t encoder_value_right = 0;
}  // namespace detail

// Interrupt for left encoder
void ENCODER_LEFT_CHANGE_EVENT() {
    uint32_t MSB = digitalRead(3);  // MSB = most significant bit
    uint32_t LSB = digitalRead(7);  // LSB = least significant bit

    uint32_t encoded = (MSB << 1) | LSB;  // converting the 2 pin value to single number
    uint32_t sum =
        (detail::last_encoded_left << 2) | encoded;  // adding it to the previous encoded value

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        detail::encoder_value_left++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        detail::encoder_value_left--;

    detail::last_encoded_left = encoded;  // store this value for next time
}

// Interrupt for right encoder
void ENCODER_RIGHT_CHANGE_EVENT() {
    int MSB = digitalRead(8);  // MSB = most significant bit
    int LSB = digitalRead(9);  // LSB = least significant bit

    int encoded = (MSB << 1) | LSB;  // converting the 2 pin value to single number
    int sum =
        (detail::last_encoded_right << 2) | encoded;  // adding it to the previous encoded value

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        detail::encoder_value_right++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        detail::encoder_value_right--;

    detail::last_encoded_right = encoded;  // store this value for next time
}

}  // namespace interrupts

/// Get the count number of the left encoder
int64_t GET_LEFT_ENCODER_VALUE() {
    return interrupts::detail::encoder_value_left;
}

/// Get the count number of the left encoder
int64_t GET_RIGHT_ENCODER_VALUE() {
    return interrupts::detail::encoder_value_right;
}

/// Initialize ir sensor array
void INITIALIZE_IR_SENSOR_ARRAY(IrSensorArrayPinConfiguration const& pin_configuration) {
    // LED_BUILTIN indicates if we are in calibration mode or not
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    qtr.setTypeAnalog();
    qtr.setSensorPins(pin_configuration.sensor_pins.data(), kMaxIrSensorArrayNumberOfLeds);
    qtr.setEmitterPins(pin_configuration.emitter_pins.at(0), pin_configuration.emitter_pins.at(1));
}

/// Calibrate ir sensor array for N iterations
void CALIBRATE_IR_SENSOR_ARRAY(size_t const iterations) {
    digitalWrite(LED_BUILTIN, HIGH);
    for (size_t idx{0U}; idx < iterations; ++idx) {
        qtr.calibrate();
    }
    digitalWrite(LED_BUILTIN, LOW);
}

/// Read sensors from an ir sensor array
bool READ_IR_SENSOR_ARRAY(uint16_t* data, size_t const data_size) {
    if (data_size == kMaxIrSensorArrayNumberOfLeds) {
        qtr.readCalibrated(data);
        return true;
    }
    return false;
}

}  // namespace arduino
}  // namespace line_follower

#endif  // LINE_FOLLOWER_DEPLOYMENT_ARDUINO_PH0_LINE_FOLLOWER_LINEFOLLOWER_H_
