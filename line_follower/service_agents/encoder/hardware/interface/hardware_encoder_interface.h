// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_SERVICE_AGENTS_ENCODER_HARDWARE_INTERFACE_HARDWARE_ENCODER_INTERFACE_H_
#define LINE_FOLLOWER_SERVICE_AGENTS_ENCODER_HARDWARE_INTERFACE_HARDWARE_ENCODER_INTERFACE_H_

#include "line_follower/external/api/encoder_interface.h"
#include "line_follower/external/types/encoder_characteristics.h"
#include "line_follower/external/types/encoder_data.h"
#include "line_follower/external/types/encoder_pin_configuration.h"

namespace line_follower {

class HardwareEncoderInterface final : public EncoderInterface {
 public:
    HardwareEncoderInterface(EncoderCharacteristics encoder_characteristics,
                             EncoderPinConfiguration pin_configuration, EncoderTag const tag)
        : EncoderInterface{tag},
          encoder_characteristics_{encoder_characteristics},
          pin_configuration_{pin_configuration} {}

    ~HardwareEncoderInterface() noexcept final = default;

    HardwareEncoderInterface(HardwareEncoderInterface const&) = delete;
    HardwareEncoderInterface(HardwareEncoderInterface&&) = delete;
    HardwareEncoderInterface& operator=(HardwareEncoderInterface const&) = delete;
    HardwareEncoderInterface& operator=(HardwareEncoderInterface&&) = delete;

    void tick(SystemTime const timestamp) override;
    bool getEncoderData(EncoderData& output) const override;
    void initialize() override;

 private:
    EncoderCharacteristics encoder_characteristics_;
    EncoderPinConfiguration pin_configuration_;
    EncoderData encoder_data_{};
    uint64_t last_encoder_value_{0U};
    int64_t time_at_last_tick_us_{0};
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_SERVICE_AGENTS_ENCODER_HARDWARE_INTERFACE_HARDWARE_ENCODER_INTERFACE_H_
