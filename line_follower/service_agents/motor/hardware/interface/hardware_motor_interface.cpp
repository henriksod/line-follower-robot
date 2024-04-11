// Copyright (c) 2023 Henrik Söderlund

#include "line_follower/service_agents/motor/hardware/interface/hardware_motor_interface.h"

#include <cstdint>

#include "line_follower/deployment/arduino/api.h"
#include "line_follower/external/types/motor_characteristics.h"
#include "line_follower/external/types/pin.h"
#include "line_follower/external/types/rotor_speed.h"

namespace line_follower {

namespace {

void setMotorDirection(MotorPinConfiguration& pin_configuration,
                       detail::MotorDirection const direction) {
    if (direction == detail::MotorDirection::kForward) {
        if (pin_configuration.reverse_direction) {
            arduino::DIGITAL_WRITE(pin_configuration.in1_pin, PinState::kHigh);
            arduino::DIGITAL_WRITE(pin_configuration.in2_pin, PinState::kLow);
        } else {
            arduino::DIGITAL_WRITE(pin_configuration.in1_pin, PinState::kLow);
            arduino::DIGITAL_WRITE(pin_configuration.in2_pin, PinState::kHigh);
        }
    } else if (direction == detail::MotorDirection::kReverse) {
        if (pin_configuration.reverse_direction) {
            arduino::DIGITAL_WRITE(pin_configuration.in1_pin, PinState::kLow);
            arduino::DIGITAL_WRITE(pin_configuration.in2_pin, PinState::kHigh);
        } else {
            arduino::DIGITAL_WRITE(pin_configuration.in1_pin, PinState::kHigh);
            arduino::DIGITAL_WRITE(pin_configuration.in2_pin, PinState::kLow);
        }
    }
}

}  // namespace

void HardwareMotorInterface::initialize() {
    arduino::SET_PIN_MODE(pin_configuration_.pwm_pin, PinMode::kOutput);
    arduino::SET_PIN_MODE(pin_configuration_.in1_pin, PinMode::kOutput);
    arduino::SET_PIN_MODE(pin_configuration_.in2_pin, PinMode::kOutput);
}

void HardwareMotorInterface::setMotorSpeed(RotorSpeed const& input) {
    if (input.revolutions_per_second < 0.0) {
        setMotorDirection(pin_configuration_, detail::MotorDirection::kReverse);
    } else {
        setMotorDirection(pin_configuration_, detail::MotorDirection::kForward);
    }

    double scaled_motor_speed{
        std::abs((255.0 / motor_characteristics_.no_load_speed) * input.revolutions_per_second)};
    arduino::ANALOG_WRITE(pin_configuration_.pwm_pin, static_cast<uint32_t>(scaled_motor_speed));
}
}  // namespace line_follower
