// Copyright (c) 2023 Henrik Söderlund

#include "line_follower/service_agents/encoder/hardware/interface/hardware_encoder_interface.h"

#include <cstdint>

#include "line_follower/deployment/arduino/api.h"
#include "line_follower/deployment/arduino/interrupts.h"
#include "line_follower/external/types/encoder_characteristics.h"
#include "line_follower/external/types/encoder_data.h"
#include "line_follower/external/types/pin.h"

namespace line_follower {

void HardwareEncoderInterface::initialize() {
    arduino::SET_PIN_MODE(pin_configuration_.enc_a_pin, PinMode::kInputPullup);
    arduino::SET_PIN_MODE(pin_configuration_.enc_b_pin, PinMode::kInputPullup);

    if (EncoderInterface::getTag() == EncoderTag::kLeft) {
        arduino::ATTACH_INTERRUPT(pin_configuration_.enc_a_pin, PinEventTrigger::kChange,
                                  arduino::interrupts::ENCODER_LEFT_CHANGE_EVENT);
        arduino::ATTACH_INTERRUPT(pin_configuration_.enc_b_pin, PinEventTrigger::kChange,
                                  arduino::interrupts::ENCODER_LEFT_CHANGE_EVENT);
    } else if (EncoderInterface::getTag() == EncoderTag::kRight) {
        arduino::ATTACH_INTERRUPT(pin_configuration_.enc_a_pin, PinEventTrigger::kChange,
                                  arduino::interrupts::ENCODER_RIGHT_CHANGE_EVENT);
        arduino::ATTACH_INTERRUPT(pin_configuration_.enc_b_pin, PinEventTrigger::kChange,
                                  arduino::interrupts::ENCODER_RIGHT_CHANGE_EVENT);
    }
}

void HardwareEncoderInterface::tick(SystemTime const timestamp) {
    if (time_at_last_tick_us_ == 0) {
        time_at_last_tick_us_ = timestamp.system_time_us;
    }

    double time_difference_seconds{(timestamp.system_time_us - time_at_last_tick_us_) * 1e-6};

    int64_t encoder_data{0U};
    if (EncoderInterface::getTag() == EncoderTag::kLeft) {
        encoder_data = arduino::GET_LEFT_ENCODER_VALUE();
    } else if (EncoderInterface::getTag() == EncoderTag::kRight) {
        encoder_data = arduino::GET_RIGHT_ENCODER_VALUE();
    }

    encoder_data_.revolutions_per_second =
        ((last_encoder_value_ - encoder_data) / encoder_characteristics_.counts_per_revolution) /
        time_difference_seconds;
    encoder_data_.step_position = encoder_data;

    time_at_last_tick_us_ = timestamp.system_time_us;
}

bool HardwareEncoderInterface::getEncoderData(EncoderData& output) const {
    output = encoder_data_;
    return true;
}

}  // namespace line_follower
