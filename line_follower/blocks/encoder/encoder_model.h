// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_ENCODER_ENCODER_MODEL_H_
#define LINE_FOLLOWER_BLOCKS_ENCODER_ENCODER_MODEL_H_

#include "line_follower/blocks/encoder/encoder_interface.h"
#include "line_follower/types/encoder_characteristics.h"
#include "line_follower/types/encoder_data.h"
#include "line_follower/types/rotor_position.h"
#include "line_follower/types/rotor_speed.h"

namespace line_follower {
class EncoderModel final : public EncoderInterface {
 public:
    explicit EncoderModel(EncoderCharacteristics encoder_characteristics)
        : encoder_characteristics_{encoder_characteristics} {}

    ~EncoderModel() noexcept final = default;

    EncoderModel(EncoderModel const&) = delete;
    EncoderModel(EncoderModel&&) = delete;
    EncoderModel& operator=(EncoderModel const&) = delete;
    EncoderModel& operator=(EncoderModel&&) = delete;

    void tick() override;
    bool getEncoderData(EncoderData& output) const override;
    void setRotorSpeed(RotorSpeed const& rotor_speed);

 private:
    EncoderCharacteristics encoder_characteristics_;
    RotorSpeed rotor_speed_{};
    RotorPosition rotor_position_{};
    int64_t time_at_last_tick_ns_{};
    double smooth_encoder_step_{};
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_ENCODER_ENCODER_MODEL_H_
