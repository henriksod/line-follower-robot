// Copyright (c) 2023 Henrik Söderlund

#include "line_follower/blocks/encoder/encoder_model.h"

#include <chrono>

namespace line_follower {
void EncoderModel::tick() {
    auto now{std::chrono::system_clock::now()};
    auto duration{now.time_since_epoch()};
    auto current_time{std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count()};

    if (time_at_last_tick_ns_ == 0) {
        time_at_last_tick_ns_ = current_time;
    }

    double time_difference_seconds{(current_time - time_at_last_tick_ns_) * 1e-9};

    smooth_encoder_step_ += (rotor_speed_.revolutions_per_second * time_difference_seconds) *
                            encoder_characteristics_.counts_per_revolution;
    rotor_position_.step_position = static_cast<uint64_t>(smooth_encoder_step_);

    time_at_last_tick_ns_ = current_time;
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
