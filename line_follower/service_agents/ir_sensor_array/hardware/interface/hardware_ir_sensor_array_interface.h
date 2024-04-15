// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_SERVICE_AGENTS_IR_SENSOR_ARRAY_HARDWARE_INTERFACE_HARDWARE_IR_SENSOR_ARRAY_INTERFACE_H_
#define LINE_FOLLOWER_SERVICE_AGENTS_IR_SENSOR_ARRAY_HARDWARE_INTERFACE_HARDWARE_IR_SENSOR_ARRAY_INTERFACE_H_

#include "line_follower/external/api/ir_sensor_array_interface.h"
#include "line_follower/external/types/ir_sensor_array_characteristics.h"
#include "line_follower/external/types/ir_sensor_array_data.h"
#include "line_follower/external/types/ir_sensor_array_pin_configuration.h"
#include "line_follower/external/types/pose.h"
#include "line_follower/external/types/system_time.h"

namespace line_follower {

namespace detail {

/// The maximum sensor reading of an ir sensor provided by the hardware interface
constexpr uint16_t kMaximumSensorReadingValue{1000U};

}  // namespace detail

class HardwareIrSensorArrayInterface final : public IrSensorArrayInterface {
    void resetIrSensorArrayData(IrSensorArrayData& ir_sensor_array_data) {
        ir_sensor_array_data.valid = false;
        for (auto& ir_sensor_data : ir_sensor_array_data.ir_sensor_readings) {
            ir_sensor_data.detected_white_surface = true;
            ir_sensor_data.intensity = 1.0;
        }
    }

 public:
    HardwareIrSensorArrayInterface(IrSensorArrayCharacteristics ir_array_characteristics,
                                   IrSensorArrayPinConfiguration pin_configuration)
        : ir_array_characteristics_{ir_array_characteristics},
          pin_configuration_{pin_configuration} {
        resetIrSensorArrayData(ir_sensor_array_data_);
    }

    ~HardwareIrSensorArrayInterface() noexcept final = default;

    HardwareIrSensorArrayInterface(HardwareIrSensorArrayInterface const&) = delete;
    HardwareIrSensorArrayInterface(HardwareIrSensorArrayInterface&&) = delete;
    HardwareIrSensorArrayInterface& operator=(HardwareIrSensorArrayInterface const&) = delete;
    HardwareIrSensorArrayInterface& operator=(HardwareIrSensorArrayInterface&&) = delete;

    void tick(SystemTime const timestamp) override;
    bool getIrSensorArrayData(IrSensorArrayData& output) const override;
    void initialize() override;
    void calibrate(size_t const iterations) override;

 private:
    IrSensorArrayCharacteristics const ir_array_characteristics_;
    IrSensorArrayPinConfiguration pin_configuration_;
    IrSensorArrayData ir_sensor_array_data_{};
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_SERVICE_AGENTS_IR_SENSOR_ARRAY_HARDWARE_INTERFACE_HARDWARE_IR_SENSOR_ARRAY_INTERFACE_H_
