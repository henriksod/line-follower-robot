// Copyright (c) 2024 Henrik Söderlund

#include <math.h>

#include <memory>
#include <sstream>

#include "line_follower/service_agents/common/logging.h"
#include "line_follower/service_agents/encoder/encoder_data_agent.h"
#include "line_follower/service_agents/ir_sensor_array/ir_sensor_array_data_agent.h"
#include "line_follower/service_agents/line_following/line_following_agent.h"
#include "line_follower/service_agents/motor/motor_signal_agent.h"
#include "line_follower/service_agents/scheduler/scheduler_agent.h"
#include "line_follower/service_agents/time/time_agent.h"
#include "line_follower/types/encoder_characteristics.h"
#include "line_follower/types/ir_sensor_array_characteristics.h"
#include "line_follower/types/line.h"
#include "line_follower/types/motor_characteristics.h"
#include "line_follower/types/pose.h"
#include "line_follower/types/robot_characteristics.h"
#include "line_follower/types/rotor_speed.h"
#include "line_follower/types/system_time.h"
#include "line_follower/types/track_lines.h"

// Only needed for simulation
#include "line_follower/blocks/dead_reckoning/dead_reckoning_model.h"
#include "line_follower/blocks/encoder/encoder_model.h"
#include "line_follower/blocks/geometry/conversion.h"
#include "line_follower/blocks/geometry/utils/rotation_utils.h"
#include "line_follower/blocks/ir_sensor_array/ir_sensor_array_model.h"
#include "line_follower/blocks/motor/motor_model.h"

namespace line_follower {
namespace {
// Update rate
constexpr uint32_t kUpdateRateMicros{10000U};

// Logging update rate
constexpr uint32_t kLoggingUpdateRateMicros{100000U};

// Convert rev/min to rev/sec
constexpr double kRevPerMinToRevPerSec{1.0 / 60.0};

// Convert kgmm to Nmm
constexpr double kKilogramMmToNewtonMm{9.80665};

// Convert microseconds to seconds
constexpr double kMicrosToSeconds{1e-6};

// The initial pose of the robot, in world coordinate system
Pose createInitialPose() {
    return {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}};
}

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
    motor_characteristics.no_load_speed = 3400 * kRevPerMinToRevPerSec;
    motor_characteristics.stall_torque.newtonmillimeters = 1.7 * kKilogramMmToNewtonMm;
    return motor_characteristics;
}

IrSensorArrayCharacteristics createIrSensorArrayCharacteristics() {
    IrSensorArrayCharacteristics ir_sensor_array_characteristics{};

    ir_sensor_array_characteristics.array_spacing = 4.0;  // mm
    ir_sensor_array_characteristics.number_of_leds = 15U;
    return ir_sensor_array_characteristics;
}

DifferentialDriveRobotCharacteristics createRobotCharacteristics() {
    DifferentialDriveRobotCharacteristics robot_characteristics{};
    robot_characteristics.wheel_radius = 0.02;
    robot_characteristics.distance_between_wheels = 0.1;
    return robot_characteristics;
}
}  // namespace

class ExampleRobot final {
 public:
    ExampleRobot()
        : scheduler_{std::make_shared<SchedulerProducerAgent>()},
          left_motor_signal_producer_agent_{},
          left_encoder_data_consumer_agent_{},
          right_motor_signal_producer_agent_{},
          right_encoder_data_consumer_agent_{},
          initial_pose_{createInitialPose()},
          time_agent_{} {
        LoggingAgent::getInstance().schedule(scheduler_, kLoggingUpdateRateMicros);

        auto dead_reckoning_model{
            std::make_unique<DeadReckoningModel>(createRobotCharacteristics(), initial_pose_)};
        dead_reckoning_model_ = dead_reckoning_model.get();
        line_following_agent_ =
            std::make_unique<LineFollowingAgent>(std::move(dead_reckoning_model));

        auto ir_sensor_array_model{
            std::make_unique<IrSensorArrayModel>(createIrSensorArrayCharacteristics())};
        ir_sensor_array_model_ = ir_sensor_array_model.get();
        ir_sensor_array_data_producer_agent_ =
            std::make_unique<IrSensorArrayDataProducerAgent>(std::move(ir_sensor_array_model));

        auto left_encoder_model{std::make_unique<EncoderModel>(createEncoderCharacteristics())};
        left_encoder_model_ = left_encoder_model.get();
        left_encoder_data_producer_agent_ =
            std::make_unique<EncoderDataProducerAgent>(std::move(left_encoder_model));

        auto right_encoder_model{std::make_unique<EncoderModel>(createEncoderCharacteristics())};
        right_encoder_model_ = right_encoder_model.get();
        right_encoder_data_producer_agent_ =
            std::make_unique<EncoderDataProducerAgent>(std::move(right_encoder_model));

        auto left_motor_model{std::make_unique<MotorModel>(createMotorCharacteristics())};
        left_motor_model_ = left_motor_model.get();
        left_motor_signal_consumer_agent_ =
            std::make_unique<MotorSignalConsumerAgent>(std::move(left_motor_model));

        auto right_motor_model{std::make_unique<MotorModel>(createMotorCharacteristics())};
        right_motor_model_ = right_motor_model.get();
        right_motor_signal_consumer_agent_ =
            std::make_unique<MotorSignalConsumerAgent>(std::move(right_motor_model));
    }

    void setup();
    void loop();

 private:
    std::shared_ptr<SchedulerProducerAgent> scheduler_;
    MotorSignalProducerAgent left_motor_signal_producer_agent_;
    EncoderDataConsumerAgent left_encoder_data_consumer_agent_;
    MotorSignalProducerAgent right_motor_signal_producer_agent_;
    EncoderDataConsumerAgent right_encoder_data_consumer_agent_;
    Pose initial_pose_;
    TimeAgent time_agent_;
    DeadReckoningModel* dead_reckoning_model_;
    EncoderModel* left_encoder_model_;
    MotorModel* left_motor_model_;
    EncoderModel* right_encoder_model_;
    MotorModel* right_motor_model_;
    IrSensorArrayModel* ir_sensor_array_model_;
    std::unique_ptr<LineFollowingAgent> line_following_agent_;
    std::unique_ptr<EncoderDataProducerAgent> left_encoder_data_producer_agent_;
    std::unique_ptr<MotorSignalConsumerAgent> left_motor_signal_consumer_agent_;
    std::unique_ptr<EncoderDataProducerAgent> right_encoder_data_producer_agent_;
    std::unique_ptr<MotorSignalConsumerAgent> right_motor_signal_consumer_agent_;
    std::unique_ptr<IrSensorArrayDataProducerAgent> ir_sensor_array_data_producer_agent_;
    TrackSegment current_track_segment_{};
    SystemTime time_at_last_update_{};
};

void ExampleRobot::setup() {
    // Set up example track segment with one line
    current_track_segment_.pose.position = {0.0, 0.0, 0.0};
    current_track_segment_.pose.rotation = {1.0, 0.0, 0.0, 0.0};
    current_track_segment_.track_lines[0].visible = true;
    current_track_segment_.track_lines[0].whiteness = 0.0;
    current_track_segment_.track_lines[0].width = 0.01;
    current_track_segment_.track_lines[0].line.start = {-10.0 * 0.004, 0.0, 0.0};
    current_track_segment_.track_lines[0].line.end = {10.0 * 0.004, 1.0, 0.0};

    // Define callbacks
    left_encoder_data_consumer_agent_.onReceiveData([this](EncoderData const& encoder_data) {
        line_following_agent_->setEncoderLeftData(encoder_data);
    });
    right_encoder_data_consumer_agent_.onReceiveData([this](EncoderData const& encoder_data) {
        line_following_agent_->setEncoderRightData(encoder_data);
    });

    time_at_last_update_ = time_agent_.getSystemTime();
    line_following_agent_->onReceiveData([this](IrSensorArrayData const& ir_sensor_array_data) {
        /// TODO: Have to move most of this to inside line following agent!
        Pose pose{line_following_agent_->getPose()};
        Pose ir_pose{line_following_agent_->getIrSensorArrayPose()};
        SystemTime current_time{time_agent_.getSystemTime()};
        auto time_diff{(current_time.system_time_us - time_at_last_update_.system_time_us) *
                       kMicrosToSeconds};
        ir_sensor_array_model_->setTrackLines(
            current_track_segment_,
            geometry::transformedPose(convert(current_track_segment_.pose.rotation),
                                      convert(current_track_segment_.pose.position), ir_pose));
        line_following_agent_->setIrSensorArrayData(ir_sensor_array_data);
        line_following_agent_->step(time_diff);

        time_at_last_update_ = time_agent_.getSystemTime();

        std::stringstream stream{};
        stream << "Read ir data: ";

        for (auto const& reading : ir_sensor_array_data.ir_sensor_readings) {
            stream << reading.detected_white_surface << " ";
        }
        stream << "\n";

        LOG_INFO("Position: (%.2f, %.2f, %.2f)", pose.position.x, pose.position.y, pose.position.z);
        LOG_INFO("Rotation: (%.2f, %.2f, %.2f, %.2f)", pose.rotation.w, pose.rotation.x,
                 pose.rotation.y, pose.rotation.z);
        LOG_INFO(stream.str());
    });

    // Attach consumers to producers
    left_motor_signal_consumer_agent_->attach(left_motor_signal_producer_agent_);
    right_motor_signal_consumer_agent_->attach(right_motor_signal_producer_agent_);
    left_encoder_data_consumer_agent_.attach(*left_encoder_data_producer_agent_);
    right_encoder_data_consumer_agent_.attach(*right_encoder_data_producer_agent_);
    line_following_agent_->attach(*ir_sensor_array_data_producer_agent_);

    // Start scheduling readings from sensors
    left_encoder_data_producer_agent_->schedule(scheduler_, kUpdateRateMicros);
    right_encoder_data_producer_agent_->schedule(scheduler_, kUpdateRateMicros);
    ir_sensor_array_data_producer_agent_->schedule(scheduler_, kUpdateRateMicros);

    // Set a constant motor speed
    MotorSignal motor_signal{};
    motor_signal.speed.revolutions_per_second = 100 * kRevPerMinToRevPerSec;
    left_motor_signal_producer_agent_.sendData(motor_signal);
    right_motor_signal_producer_agent_.sendData(motor_signal);
}

void ExampleRobot::loop() {
    // Step the software using the scheduler
    scheduler_->tick();
    // In simulation we have to transfer the motor rotations to the encoders manually
    left_encoder_model_->setRotorSpeed(left_motor_model_->getMotorSpeed());
    right_encoder_model_->setRotorSpeed(right_motor_model_->getMotorSpeed());
}
}  // namespace line_follower

int main(int argc, char* argv[]) {
    line_follower::ExampleRobot robot{};

    robot.setup();

    while (true) robot.loop();
    return 0;
}
