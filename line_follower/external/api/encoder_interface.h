// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_API_ENCODER_INTERFACE_H_
#define LINE_FOLLOWER_EXTERNAL_API_ENCODER_INTERFACE_H_

#include "line_follower/external/types/encoder_data.h"

namespace line_follower {
class EncoderInterface {
 public:
    EncoderInterface() {}

    virtual ~EncoderInterface() noexcept = default;

    EncoderInterface(EncoderInterface const&) = delete;
    EncoderInterface(EncoderInterface&&) = delete;
    EncoderInterface& operator=(EncoderInterface const&) = delete;
    EncoderInterface& operator=(EncoderInterface&&) = delete;

    virtual void tick() = 0;
    virtual bool getEncoderData(EncoderData& output) const = 0;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_API_ENCODER_INTERFACE_H_
