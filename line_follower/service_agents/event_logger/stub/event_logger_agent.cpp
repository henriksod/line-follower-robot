// Copyright (c) 2024 Henrik Söderlund

#include "line_follower/external/api/event_logger_agent.h"

#include <string>

#include "line_follower/external/api/common.h"
#include "line_follower/external/types/ir_sensor_array_data.h"
#include "line_follower/external/types/line_follower_event_state.h"

namespace line_follower {

template <>
void EventLoggerAgent<IrSensorArrayData, LineFollowerEventState>::workerThread() {}

template <>
void EventLoggerAgent<IrSensorArrayData, LineFollowerEventState>::logEvent(
    std::string const& filename, LineFollowerEventState const event_state) {
    static_cast<void>(filename);
    static_cast<void>(event_state);
}

}  // namespace line_follower
