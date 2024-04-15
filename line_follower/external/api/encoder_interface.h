// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_API_ENCODER_INTERFACE_H_
#define LINE_FOLLOWER_EXTERNAL_API_ENCODER_INTERFACE_H_

#include "line_follower/external/types/encoder_data.h"
#include "line_follower/external/types/encoder_tag.h"
#include "line_follower/external/types/system_time.h"

namespace line_follower {
class EncoderInterface {
 public:
    explicit EncoderInterface(EncoderTag const tag) : tag_{tag} {}

    virtual ~EncoderInterface() noexcept = default;

    EncoderInterface(EncoderInterface const&) = delete;
    EncoderInterface(EncoderInterface&&) = delete;
    EncoderInterface& operator=(EncoderInterface const&) = delete;
    EncoderInterface& operator=(EncoderInterface&&) = delete;

    virtual void tick(SystemTime const timestamp) = 0;
    virtual bool getEncoderData(EncoderData& output) const = 0;
    virtual void initialize() = 0;

    EncoderTag getTag() const { return tag_; }

 private:
    EncoderTag tag_;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_API_ENCODER_INTERFACE_H_
