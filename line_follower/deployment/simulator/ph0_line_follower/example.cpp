// Copyright (c) 2024 Henrik Söderlund

#include <bits/types/sig_atomic_t.h>
#include <math.h>

#include <csignal>
#include <iostream>
#include <memory>
#include <sstream>

// clang-format off
#include <cxxopts.hpp>
// clang-format on

#include "line_follower/external/api/encoder_data_agent.h"
#include "line_follower/external/api/ir_sensor_array_data_agent.h"
#include "line_follower/external/api/line_following_agent.h"
#include "line_follower/external/api/logging.h"
#include "line_follower/external/api/logging_agent.h"
#include "line_follower/external/api/motor_signal_agent.h"
#include "line_follower/external/api/scheduler_agent.h"
#include "line_follower/external/types/encoder_characteristics.h"
#include "line_follower/external/types/encoder_tag.h"
#include "line_follower/external/types/ir_sensor_array_characteristics.h"
#include "line_follower/external/types/line.h"
#include "line_follower/external/types/line_follower_event_state.h"
#include "line_follower/external/types/line_following_statistics.h"
#include "line_follower/external/types/logging_verbosity_level.h"
#include "line_follower/external/types/motor_characteristics.h"
#include "line_follower/external/types/motor_signal.h"
#include "line_follower/external/types/pose.h"
#include "line_follower/external/types/robot_characteristics.h"
#include "line_follower/external/types/rotor_speed.h"
#include "line_follower/external/types/system_time.h"
#include "line_follower/external/types/track_lines.h"

// Only needed for simulation
#include "line_follower/blocks/dead_reckoning/dead_reckoning_model.h"
#include "line_follower/blocks/encoder/encoder_model.h"
#include "line_follower/blocks/geometry/conversion.h"
#include "line_follower/blocks/geometry/utils/rotation_utils.h"
#include "line_follower/blocks/ir_sensor_array/ir_sensor_array_model.h"
#include "line_follower/blocks/line_following/simple_line_following_model.h"
#include "line_follower/blocks/motor/motor_model.h"
#include "line_follower/blocks/robot_geometry/robot_geometry.h"
#include "line_follower/blocks/utilities/calibration_loader.h"
#include "line_follower/blocks/utilities/event_logger.h"
#include "line_follower/blocks/utilities/should_exit.h"
#include "line_follower/blocks/utilities/track_loader.h"

namespace {

/// Intended to take care of exit requests by setting termination flag
extern "C" void signalHandler(int /*signo*/) {
    should_exit = 1;
    std::cerr << "Received signal to exit. Exiting..." << std::endl;
}

}  // namespace

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
// constexpr double kMicrosToSeconds{1e-6};

EncoderCharacteristics createEncoderCharacteristics() {
    EncoderCharacteristics encoder_characteristics{};

    encoder_characteristics.counts_per_revolution = 25.0;
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
    ir_sensor_array_characteristics.line_detected_threshold = 0.75;
    return ir_sensor_array_characteristics;
}

DifferentialDriveRobotCharacteristics createRobotCharacteristics() {
    DifferentialDriveRobotCharacteristics robot_characteristics{};
    robot_characteristics.wheel_radius = 0.01;
    robot_characteristics.distance_between_wheels = 0.2;
    return robot_characteristics;
}

LineFollowingCharacteristics createLineFollowingCharacteristics() {
    LineFollowingCharacteristics line_following_characteristics{};

    line_following_characteristics.pid_speed_parameters.proportional_gain = 1.0;
    line_following_characteristics.pid_speed_parameters.integral_gain = 0.0;
    line_following_characteristics.pid_speed_parameters.derivative_gain = 0.0;
    line_following_characteristics.pid_speed_parameters.min_value = 0.0;
    line_following_characteristics.pid_speed_parameters.max_value = 100.0;

    line_following_characteristics.pid_steer_parameters.proportional_gain = 2.0;
    line_following_characteristics.pid_steer_parameters.integral_gain = 0.0;
    line_following_characteristics.pid_steer_parameters.derivative_gain = 0.0;
    line_following_characteristics.pid_steer_parameters.min_value = -2.0;
    line_following_characteristics.pid_steer_parameters.max_value = 2.0;

    line_following_characteristics.measurement_noise = 0.1;
    line_following_characteristics.position_state_noise = 0.1;
    line_following_characteristics.position_derivative_state_noise = 0.01;

    line_following_characteristics.max_forward_velocity = 0.05;
    line_following_characteristics.turning_speed_ratio = 0.0;

    line_following_characteristics.sharp_turn_forward_velocity = 0.1;
    line_following_characteristics.sharp_turn_angular_velocity = 2.0;

    return line_following_characteristics;
}

}  // namespace

class ExampleRobot final {
    bool initializeLogging_(LoggingVerbosityLevel const verbosity) {
        LoggingAgent::getInstance().setVerbosityLevel(verbosity);
        LoggingAgent::getInstance().schedule(*scheduler_, kLoggingUpdateRateMicros);
        return true;
    }

    void detectSimulationTimeout_(IrSensorArrayData const& ir_sensor_array_data,
                                  EncoderData const& left_encoder_data,
                                  EncoderData const& right_encoder_data) {
        // Check if the robot is stuck
        if (simulation_timeout_iteration_ > 100) {
            LOG_ERROR("Robot is out of bounds. Stopping simulation.");
            should_exit = 1;
        }

        if (fabs(left_encoder_data.revolutions_per_second) < 0.01 &&
            fabs(right_encoder_data.revolutions_per_second) < 0.01) {
            ++simulation_timeout_iteration_;
        } else if (ir_sensor_array_data.valid) {
            for (auto const& reading : ir_sensor_array_data.ir_sensor_readings) {
                if (!reading.detected_white_surface) {
                    simulation_timeout_iteration_ = 0U;
                    return;
                }
            }
            ++simulation_timeout_iteration_;
        } else {
            simulation_timeout_iteration_ = 0U;
        }
    }

 public:
    ExampleRobot(std::string const& scenario_file, std::string const& calibration_file,
                 std::string const& event_file, LoggingVerbosityLevel const verbosity)
        : scheduler_{std::make_shared<SchedulerProducerAgent>()},
          logging_initialized_{initializeLogging_(verbosity)},
          event_logger_{event_file},
          left_encoder_data_consumer_agent_{},
          right_encoder_data_consumer_agent_{} {
        // Load the track segments
        if (!line_follower::track_loader::loadScenarioFromJson(scenario_file, track_segments_,
                                                               initial_pose_)) {
            LOG_FATAL_ABORT("Could not load scenario!");
        }
        // Load calibration data
        LineFollowingCharacteristics line_following_characteristics{
            createLineFollowingCharacteristics()};
        if (!line_follower::calibration_loader::loadCalibrationFromJson(
                calibration_file, line_following_characteristics)) {
            LOG_INFO("Using default calibration data.");
        }
        // Convert initial pose from world coordinates to robot coordinates
        initial_pose_ =
            geometry::transformedPose(robot_geometry::kWorldToRobotRotation,
                                      robot_geometry::kWorldToRobotPosition, initial_pose_);

        auto dead_reckoning_model{
            std::make_unique<DeadReckoningModel>(createRobotCharacteristics(), initial_pose_)};
        dead_reckoning_model_ = dead_reckoning_model.get();

        auto line_following_model{std::make_unique<SimpleLineFollowingModel>(
            line_following_characteristics, std::move(dead_reckoning_model))};
        line_following_model_ = line_following_model.get();

        line_following_agent_ =
            std::make_unique<LineFollowingAgent>(std::move(line_following_model));

        auto ir_sensor_array_model{
            std::make_unique<IrSensorArrayModel>(createIrSensorArrayCharacteristics())};
        ir_sensor_array_model_ = ir_sensor_array_model.get();
        ir_sensor_array_data_producer_agent_ =
            std::make_unique<IrSensorArrayDataProducerAgent>(std::move(ir_sensor_array_model));

        auto left_encoder_model{
            std::make_unique<EncoderModel>(createEncoderCharacteristics(), EncoderTag::kLeft)};
        left_encoder_model_ = left_encoder_model.get();
        left_encoder_data_producer_agent_ =
            std::make_unique<EncoderDataProducerAgent>(std::move(left_encoder_model));

        auto right_encoder_model{
            std::make_unique<EncoderModel>(createEncoderCharacteristics(), EncoderTag::kRight)};
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
    bool logging_initialized_;
    EventLogger<LineFollowerEventState> event_logger_;
    EncoderDataConsumerAgent left_encoder_data_consumer_agent_;
    EncoderDataConsumerAgent right_encoder_data_consumer_agent_;
    Pose initial_pose_;
    DeadReckoningModel* dead_reckoning_model_;
    SimpleLineFollowingModel* line_following_model_;
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
    std::vector<line_follower::TrackSegment> track_segments_{};
    std::size_t simulation_timeout_iteration_{0U};
};

void ExampleRobot::setup() {
    // Define callbacks
    left_encoder_data_consumer_agent_.onReceiveData([this](EncoderData const& encoder_data) {
        LOG_INFO("Left encoder data: %.2f rev/s", encoder_data.revolutions_per_second);
        line_following_agent_->setEncoderLeftData(encoder_data);
    });
    right_encoder_data_consumer_agent_.onReceiveData([this](EncoderData const& encoder_data) {
        LOG_INFO("Right encoder data: %.2f rev/s", encoder_data.revolutions_per_second);
        line_following_agent_->setEncoderRightData(encoder_data);
    });

    line_following_agent_->onReceiveData([this](IrSensorArrayData const& ir_sensor_array_data) {
        Pose pose{line_following_agent_->getPose()};
        Pose ir_pose{line_following_agent_->getIrSensorArrayPose()};
        ir_sensor_array_model_->setTrackLines(track_segments_, ir_pose);

        line_following_agent_->setIrSensorArrayData(ir_sensor_array_data);
        line_following_agent_->step();

        LineFollowerEventState event_state{};
        event_state.timestamp = ir_sensor_array_data.timestamp;
        left_encoder_model_->getEncoderData(event_state.left_encoder_data_input);
        right_encoder_model_->getEncoderData(event_state.right_encoder_data_input);
        event_state.global_pose = pose;
        event_state.ir_pose = ir_pose;
        event_state.line_following_statistics = line_following_agent_->getStatistics();
        event_logger_.logEvent(event_state);

        // Detect simulation timeout
        detectSimulationTimeout_(ir_sensor_array_data, event_state.left_encoder_data_input,
                                 event_state.right_encoder_data_input);

        LOG_INFO("Line following statistics: %.2f %.2f %.2f",
                 event_state.line_following_statistics.tracking_error,
                 event_state.line_following_statistics.average_speed,
                 event_state.line_following_statistics.time_spent_on_line);

        std::stringstream stream{};
        stream << "Read ir data: ";
        for (auto const& reading : ir_sensor_array_data.ir_sensor_readings) {
            stream << reading.detected_white_surface << " ";
        }
        stream << "\n";

        LOG_INFO("Position: (%.2f, %.2f, %.2f)", pose.position.x, pose.position.y, pose.position.z);
        LOG_INFO("Rotation: (%.2f, %.2f, %.2f, %.2f)", pose.rotation.w, pose.rotation.x,
                 pose.rotation.y, pose.rotation.z);
        LOG_INFO("IR Position: (%.2f, %.2f, %.2f)", ir_pose.position.x, ir_pose.position.y,
                 ir_pose.position.z);
        LOG_INFO("IR Rotation: (%.2f, %.2f, %.2f, %.2f)", ir_pose.rotation.w, ir_pose.rotation.x,
                 ir_pose.rotation.y, ir_pose.rotation.z);
        LOG_INFO(stream.str());
    });

    // Attach consumers to producers
    left_encoder_data_consumer_agent_.attach(*left_encoder_data_producer_agent_);
    right_encoder_data_consumer_agent_.attach(*right_encoder_data_producer_agent_);
    line_following_agent_->attach(*ir_sensor_array_data_producer_agent_);

    // Attach motors to line follower agent
    line_following_agent_->attachMotorLeft(*left_motor_signal_consumer_agent_);
    line_following_agent_->attachMotorRight(*right_motor_signal_consumer_agent_);

    // Start scheduling readings from sensors
    left_encoder_data_producer_agent_->schedule(*scheduler_, kUpdateRateMicros);
    right_encoder_data_producer_agent_->schedule(*scheduler_, kUpdateRateMicros);
    ir_sensor_array_data_producer_agent_->schedule(*scheduler_, kUpdateRateMicros);
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
    // Register signal handler for SIGINT (Ctrl+C)
    static_cast<void>(signal(SIGINT, signalHandler));
    static_cast<void>(signal(SIGTERM, signalHandler));

    cxxopts::Options options("Example Line Follower Simulator", "Runs line follower simulation");

    options.add_options()("h,help", "Print usage")(
        "scenario", "Scenario file containing the track to run", cxxopts::value<std::string>())(
        "verbosity", "The verbosity of logging (debug, info, warn, error)",
        cxxopts::value<std::string>()->default_value("info"))(
        "events_log_json", "Json file to store event logs",
        cxxopts::value<std::string>()->default_value(""))(
        "calibration", "Json file to store calibration values",
        cxxopts::value<std::string>()->default_value(""));

    auto result = options.parse(argc, argv);

    if (result.count("help") > 0) {
        std::cout << options.help() << std::endl;
        return 0;
    }

    if (result.count("scenario") == 0) {
        std::cerr << "No scenario file provided!\n";
        return 1;
    }

    std::string events_log_json{result["events_log_json"].as<std::string>()};
    std::string calibration_json{result["calibration"].as<std::string>()};

    auto scenario_file = result["scenario"].as<std::string>();
    auto verbosity = result["verbosity"].as<std::string>();
    auto verbosity_level = line_follower::parseVerbosityLevel(verbosity);

    line_follower::ExampleRobot robot{scenario_file, calibration_json, events_log_json,
                                      verbosity_level};

    robot.setup();

    while (!should_exit) robot.loop();
    std::cerr << "Exiting..." << std::endl;
    return 0;
}
