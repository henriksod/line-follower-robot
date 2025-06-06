// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_ENCODER_ENCODER_MODEL_H_
#define LINE_FOLLOWER_BLOCKS_ENCODER_ENCODER_MODEL_H_

#include "line_follower/external/api/encoder_interface.h"
#include "line_follower/external/types/encoder_characteristics.h"
#include "line_follower/external/types/encoder_data.h"
#include "line_follower/external/types/encoder_tag.h"
#include "line_follower/external/types/rotor_position.h"
#include "line_follower/external/types/rotor_speed.h"
#include "line_follower/external/types/system_time.h"

namespace line_follower {
class EncoderModel final : public EncoderInterface {
 public:
    EncoderModel(EncoderCharacteristics encoder_characteristics, EncoderTag const tag)
        : EncoderInterface{tag}, encoder_characteristics_{encoder_characteristics} {}

    ~EncoderModel() noexcept final = default;

    EncoderModel(EncoderModel const&) = delete;
    EncoderModel(EncoderModel&&) = delete;
    EncoderModel& operator=(EncoderModel const&) = delete;
    EncoderModel& operator=(EncoderModel&&) = delete;

    void tick(SystemTime const timestamp) override;
    bool getEncoderData(EncoderData& output) const override;
    void setRotorSpeed(RotorSpeed const& rotor_speed);
    void initialize() override {}

 private:
    EncoderCharacteristics encoder_characteristics_;
    RotorSpeed rotor_speed_{};
    RotorPosition rotor_position_{};
    uint64_t time_at_last_tick_us_{};
    double smooth_encoder_step_{};
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_ENCODER_ENCODER_MODEL_H_
