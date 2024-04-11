// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_DEPLOYMENT_ARDUINO_PH0_LINE_FOLLOWER_LINEFOLLOWER_H_
#define LINE_FOLLOWER_DEPLOYMENT_ARDUINO_PH0_LINE_FOLLOWER_LINEFOLLOWER_H_

#include <Arduino.h>

#include "line_follower/external/api/encoder_data_agent.h"
#include "line_follower/external/api/ir_sensor_array_data_agent.h"
#include "line_follower/external/api/line_following_agent.h"
#include "line_follower/external/api/logging.h"
#include "line_follower/external/api/logging_agent.h"
#include "line_follower/external/api/motor_signal_agent.h"
#include "line_follower/external/api/scheduler_agent.h"
#include "line_follower/external/api/time_agent.h"

namespace line_follower {
namespace arduino {

uint32_t GET_SYSTEM_TIME_MICROS() {
    return micros();
}

void PRINT_MESSAGE(char* message, size_t length) {
    Serial.write(message, length);
}

}  // namespace arduino
}  // namespace line_follower

#endif  // LINE_FOLLOWER_DEPLOYMENT_ARDUINO_PH0_LINE_FOLLOWER_LINEFOLLOWER_H_
