// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_SERVICE_AGENTS_MOTOR_HARDWARE_INTERFACE_HARDWARE_MOTOR_INTERFACE_H_
#define LINE_FOLLOWER_SERVICE_AGENTS_MOTOR_HARDWARE_INTERFACE_HARDWARE_MOTOR_INTERFACE_H_

#include "line_follower/external/api/motor_interface.h"
#include "line_follower/external/types/motor_characteristics.h"
#include "line_follower/external/types/motor_pin_configuration.h"
#include "line_follower/external/types/rotor_speed.h"

namespace line_follower {

namespace detail {

/// A motor rotation direction
enum class MotorDirection : uint32_t { kForward = 0U, kReverse };

}  // namespace detail

class HardwareMotorInterface final : public MotorInterface {
 public:
    HardwareMotorInterface(MotorCharacteristics motor_characteristics,
                           MotorPinConfiguration pin_configuration)
        : motor_characteristics_{motor_characteristics}, pin_configuration_{pin_configuration} {
        initialize();
    }

    ~HardwareMotorInterface() noexcept final = default;

    HardwareMotorInterface(HardwareMotorInterface const&) = delete;
    HardwareMotorInterface(HardwareMotorInterface&&) = delete;
    HardwareMotorInterface& operator=(HardwareMotorInterface const&) = delete;
    HardwareMotorInterface& operator=(HardwareMotorInterface&&) = delete;

    void setMotorSpeed(RotorSpeed const& input) override;

 private:
    MotorCharacteristics motor_characteristics_;
    MotorPinConfiguration pin_configuration_;

    void initialize();
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_SERVICE_AGENTS_MOTOR_HARDWARE_INTERFACE_HARDWARE_MOTOR_INTERFACE_H_
