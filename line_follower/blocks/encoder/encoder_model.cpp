// Copyright (c) 2023 Henrik Söderlund

#include "line_follower/blocks/encoder/encoder_model.h"

#include <chrono>

namespace line_follower {

void EncoderModel::tick(SystemTime const timestamp) {
    if (time_at_last_tick_us_ == 0) {
        time_at_last_tick_us_ = timestamp.system_time_us;
    }

    double time_difference_seconds{(timestamp.system_time_us - time_at_last_tick_us_) * 1e-6};

    smooth_encoder_step_ += (rotor_speed_.revolutions_per_second * time_difference_seconds) *
                            encoder_characteristics_.counts_per_revolution;
    rotor_position_.step_position = static_cast<uint64_t>(smooth_encoder_step_);

    time_at_last_tick_us_ = timestamp.system_time_us;
}

bool EncoderModel::getEncoderData(EncoderData& output) const {
    output.revolutions_per_second = rotor_speed_.revolutions_per_second;
    output.step_position = rotor_position_.step_position;
    return true;
}

void EncoderModel::setRotorSpeed(RotorSpeed const& rotor_speed) {
    rotor_speed_ = rotor_speed;
}
}  // namespace line_follower
