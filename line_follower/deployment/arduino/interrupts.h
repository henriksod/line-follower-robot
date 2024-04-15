// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_DEPLOYMENT_ARDUINO_INTERRUPTS_H_
#define LINE_FOLLOWER_DEPLOYMENT_ARDUINO_INTERRUPTS_H_

namespace line_follower {
namespace arduino {
namespace interrupts {

// Interrupt for left encoder
extern void ENCODER_LEFT_CHANGE_EVENT();

// Interrupt for right encoder
extern void ENCODER_RIGHT_CHANGE_EVENT();

}  // namespace interrupts
}  // namespace arduino
}  // namespace line_follower

#endif  // LINE_FOLLOWER_DEPLOYMENT_ARDUINO_INTERRUPTS_H_
