// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_EXTERNAL_API_IR_SENSOR_ARRAY_INTERFACE_H_
#define LINE_FOLLOWER_EXTERNAL_API_IR_SENSOR_ARRAY_INTERFACE_H_

#include "line_follower/external/types/ir_sensor_array_data.h"
#include "line_follower/external/types/system_time.h"

namespace line_follower {
class IrSensorArrayInterface {
 public:
    IrSensorArrayInterface() {}

    virtual ~IrSensorArrayInterface() noexcept = default;

    IrSensorArrayInterface(IrSensorArrayInterface const&) = delete;
    IrSensorArrayInterface(IrSensorArrayInterface&&) = delete;
    IrSensorArrayInterface& operator=(IrSensorArrayInterface const&) = delete;
    IrSensorArrayInterface& operator=(IrSensorArrayInterface&&) = delete;

    virtual void tick(SystemTime const timestamp) = 0;
    virtual bool getIrSensorArrayData(IrSensorArrayData& output) const = 0;
    virtual void initialize() = 0;
    virtual void calibrate(size_t const iterations) = 0;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_EXTERNAL_API_IR_SENSOR_ARRAY_INTERFACE_H_
