// Copyright (c) 2024 Henrik Söderlund

#include <math.h>
#include <memory>
#include <iostream>

#include "line_follower/types/system_time.h"
#include "line_follower/service_agents/time/time_agent.h"
#include "line_follower/service_agents/scheduler/scheduler_agent.h"
#include "line_follower/types/rotor_speed.h"
#include "line_follower/types/pose.h"
#include "line_follower/types/line.h"
#include "line_follower/types/track_lines.h"
#include "line_follower/types/ir_sensor_array_characteristics.h"
#include "line_follower/service_agents/ir_sensor_array/ir_sensor_array_data_agent.h"
#include "line_follower/types/encoder_characteristics.h"
#include "line_follower/service_agents/encoder/encoder_data_agent.h"
#include "line_follower/types/motor_characteristics.h"
#include "line_follower/service_agents/motor/motor_signal_agent.h"

// Only needed for simulation
#include "line_follower/blocks/encoder/encoder_model.h"
#include "line_follower/blocks/ir_sensor_array/ir_sensor_array_model.h"
#include "line_follower/blocks/motor/motor_model.h"


namespace line_follower {
namespace {
// Update rate
constexpr uint32_t kUpdateRateMicros{ 10000U };

// Convert rev/min to rev/sec
constexpr double kRevPerMinToRevPerSec{ 1.0 / 60.0 };

// Convert kgmm to Nmm
constexpr double kKilogramMmToNewtonMm{ 9.80665 };

// Convert microseconds to seconds
constexpr double kMicrosToSeconds{ 1e-6 };

constexpr double kRobotWheelRadiusMeters{ 0.02 };

EncoderCharacteristics createEncoderCharacteristics() {
  EncoderCharacteristics encoder_characteristics{};

  encoder_characteristics.counts_per_revolution = 25U;
  return encoder_characteristics;
}

MotorCharacteristics createMotorCharacteristics() {
  MotorCharacteristics motor_characteristics{};

  // https://www.pololu.com/file/0J1487/pololu-micro-metal-gearmotors_rev-5-1.pdf
  // 10:1 micro metal gearmotor HPCB 12V
  motor_characteristics.gear_ratio.ratio = 0.1;  // Resembles a 10:1 ratio
  motor_characteristics.no_load_speed    = 3400 *
                                           kRevPerMinToRevPerSec;
  motor_characteristics.stall_torque.newtonmillimeters = 1.7 *
                                                         kKilogramMmToNewtonMm;
  return motor_characteristics;
}

IrSensorArrayCharacteristics createIrSensorArrayCharacteristics() {
  IrSensorArrayCharacteristics ir_sensor_array_characteristics{};

  ir_sensor_array_characteristics.array_spacing  = 4.0;    // mm
  ir_sensor_array_characteristics.number_of_leds = 15U;
  return ir_sensor_array_characteristics;
}
}  // namespace

class ExampleRobot final {
 public:
  ExampleRobot()
    : scheduler_{std::make_shared<SchedulerProducerAgent>()},
    motor_signal_producer_agent_{},
    ir_sensor_array_data_consumer_agent_{},
    encoder_data_consumer_agent_{},
    pose_{},
    time_agent_{} {
    auto encoder_model{ std::make_unique<EncoderModel>(createEncoderCharacteristics()) };

    encoder_model_               = encoder_model.get();
    encoder_data_producer_agent_ =
      std::make_unique<EncoderDataProducerAgent>(std::move(encoder_model));

    auto ir_sensor_array_model{ std::make_unique<IrSensorArrayModel>(
                                  createIrSensorArrayCharacteristics()) };
    ir_sensor_array_model_               = ir_sensor_array_model.get();
    ir_sensor_array_data_producer_agent_ =
      std::make_unique<IrSensorArrayDataProducerAgent>(std::move(ir_sensor_array_model));

    auto motor_model{ std::make_unique<MotorModel>(createMotorCharacteristics()) };
    motor_model_                 = motor_model.get();
    motor_signal_consumer_agent_ =
      std::make_unique<MotorSignalConsumerAgent>(std::move(motor_model));
  }

  void setup();
  void loop();
 private:
  std::shared_ptr<SchedulerProducerAgent>scheduler_;
  MotorSignalProducerAgent motor_signal_producer_agent_;
  IrSensorArrayDataConsumerAgent ir_sensor_array_data_consumer_agent_;
  EncoderDataConsumerAgent encoder_data_consumer_agent_;
  Pose pose_;
  TimeAgent time_agent_;
  EncoderModel *encoder_model_;
  IrSensorArrayModel *ir_sensor_array_model_;
  MotorModel *motor_model_;
  std::unique_ptr<EncoderDataProducerAgent>encoder_data_producer_agent_;
  std::unique_ptr<MotorSignalConsumerAgent>motor_signal_consumer_agent_;
  std::unique_ptr<IrSensorArrayDataProducerAgent>ir_sensor_array_data_producer_agent_;
  SystemTime time_at_last_encoder_update_{};
  TrackSegment current_track_segment_{};
};

void ExampleRobot::setup() {
  // Set up start pose
  pose_.position = { 0.0, 0.0, 0.0 };
  pose_.rotation = { 1.0, 0.0, 0.0, 0.0 };

  // Set up example track segment with one line
  current_track_segment_.pose.position             = { 0.0, 0.0, 0.0 };
  current_track_segment_.pose.rotation             = { 1.0, 0.0, 0.0, 0.0 };
  current_track_segment_.track_lines[0].visible    = true;
  current_track_segment_.track_lines[0].whiteness  = 0.0;
  current_track_segment_.track_lines[0].width      = 0.01;
  current_track_segment_.track_lines[0].line.start = { -20.0 * 0.004, 0.0, 0.0 };
  current_track_segment_.track_lines[0].line.end   = { 20.0 * 0.004, 1.0, 0.0 };

  // Define callbacks
  time_at_last_encoder_update_ = time_agent_.getSystemTime();
  encoder_data_consumer_agent_.onReceiveData(
    [this] (EncoderData const& encoder_data)
    {
      SystemTime current_time{ time_agent_.getSystemTime() };
      auto time_diff{ (current_time.system_time_us - time_at_last_encoder_update_.system_time_us) *
                      kMicrosToSeconds };
      pose_.position.y += (2.0 * M_PI * kRobotWheelRadiusMeters *
                           encoder_data.revolutions_per_second) * time_diff;
      ir_sensor_array_model_->setTrackLines(current_track_segment_, pose_);
      time_at_last_encoder_update_ = time_agent_.getSystemTime();

      /// TODO: Here we would update the pose based on encoders and a dead reckoning model
    });

  ir_sensor_array_data_consumer_agent_.onReceiveData(
    [this] (IrSensorArrayData const& ir_sensor_array_data)
    {
      std::cerr << "Position: (" << pose_.position.x << ", " << pose_.position.y << ", " <<
        pose_.position.z << ")";
      std::cerr << "\tRead ir data: ";

      for (auto const& reading : ir_sensor_array_data.ir_sensor_readings) {
        std::cerr << reading.detected_white_surface << " ";
      }
      std::cerr << "\n";

      /// TODO: Here we could implement line following logic!
    });

  // Attach consumers to producers
  motor_signal_consumer_agent_->attach(motor_signal_producer_agent_);
  ir_sensor_array_data_consumer_agent_.attach(*ir_sensor_array_data_producer_agent_);
  encoder_data_consumer_agent_.attach(*encoder_data_producer_agent_);

  // Start scheduling readings from sensors
  encoder_data_producer_agent_->schedule(scheduler_, kUpdateRateMicros);
  ir_sensor_array_data_producer_agent_->schedule(scheduler_,
                                                 kUpdateRateMicros);

  // Set a constant motor speed
  MotorSignal motor_signal{};
  motor_signal.speed.revolutions_per_second = 100 * kRevPerMinToRevPerSec;
  motor_signal_producer_agent_.sendData(motor_signal);
}

void ExampleRobot::loop() {
  scheduler_->tick();
  encoder_model_->setRotorSpeed(motor_model_->getMotorSpeed());
}
}  // namespace line_follower

int main(int argc, char *argv[]) {
  line_follower::ExampleRobot robot{};

  robot.setup();

  while (true) robot.loop();
  return 0;
}
