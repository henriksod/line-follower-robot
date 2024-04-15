// Copyright (c) 2023 Henrik Söderlund

#include "line_follower/service_agents/ir_sensor_array/hardware/interface/hardware_ir_sensor_array_interface.h"

#include <array>
#include <cstdint>

#include "line_follower/deployment/arduino/api.h"
#include "line_follower/external/types/ir_sensor_array_characteristics.h"
#include "line_follower/external/types/ir_sensor_array_data.h"
#include "line_follower/external/types/ir_sensor_array_pin_configuration.h"
#include "line_follower/external/types/system_time.h"

namespace line_follower {

void HardwareIrSensorArrayInterface::initialize() {
    arduino::INITIALIZE_IR_SENSOR_ARRAY(pin_configuration_);
}

void HardwareIrSensorArrayInterface::calibrate(size_t const iterations) {
    arduino::CALIBRATE_IR_SENSOR_ARRAY(iterations);
}

void HardwareIrSensorArrayInterface::tick(SystemTime const timestamp) {
    std::array<uint16_t, kMaxIrSensorArrayNumberOfLeds> data{};
    if (arduino::READ_IR_SENSOR_ARRAY(data.data(), data.size())) {
        for (size_t idx{0U}; idx < ir_array_characteristics_.number_of_leds; ++idx) {
            auto& ir_sensor = ir_sensor_array_data_.ir_sensor_readings[idx];
            ir_sensor.intensity =
                1.0 - static_cast<double>(data[idx] / detail::kMaximumSensorReadingValue);
            ir_sensor.detected_white_surface =
                1.0 - ir_sensor.intensity < ir_array_characteristics_.line_detected_threshold;
        }
        ir_sensor_array_data_.valid = true;
    } else {
        ir_sensor_array_data_.valid = false;
    }
    ir_sensor_array_data_.number_of_leds = ir_array_characteristics_.number_of_leds;
    ir_sensor_array_data_.timestamp = timestamp;
}

bool HardwareIrSensorArrayInterface::getIrSensorArrayData(IrSensorArrayData& output) const {
    output = ir_sensor_array_data_;
    return ir_sensor_array_data_.valid;
}

}  // namespace line_follower
