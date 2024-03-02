// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_MOTOR_MOTOR_MODEL_H_
#define LINE_FOLLOWER_BLOCKS_MOTOR_MOTOR_MODEL_H_

#include "line_follower/blocks/motor/motor_interface.h"
#include "line_follower/types/motor_characteristics.h"
#include "line_follower/types/rotor_speed.h"
#include "line_follower/types/torque.h"

namespace line_follower {
class MotorModel final : public MotorInterface {
 public:
    explicit MotorModel(MotorCharacteristics motor_characteristics)
        : motor_characteristics_{motor_characteristics} {}

    ~MotorModel() noexcept final = default;

    MotorModel(MotorModel const&) = delete;
    MotorModel(MotorModel&&) = delete;
    MotorModel& operator=(MotorModel const&) = delete;
    MotorModel& operator=(MotorModel&&) = delete;

    Torque getMotorTorque() const;
    RotorSpeed getMotorSpeed() const;
    void setMotorSpeed(RotorSpeed const& input) override;

 private:
    MotorCharacteristics motor_characteristics_;
    RotorSpeed target_rotor_speed_{};
    Torque rotor_torque_{};
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_MOTOR_MOTOR_MODEL_H_
