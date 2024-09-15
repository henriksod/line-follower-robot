// Copyright (c) 2024 Henrik Söderlund

#include <Arduino.h>
#include <LineFollower.h>

#include <array>
#include <sstream>

namespace line_follower {

namespace {
/// Calibration iterations for ir sensor array
constexpr size_t kCalibrationIterations{400U};

/// Update rate
constexpr uint32_t kUpdateRateMicros{10000U};

/// Logging update rate
constexpr uint32_t kLoggingUpdateRateMicros{100000U};

/// Convert rev/min to rev/sec
constexpr double kRevPerMinToRevPerSec{1.0 / 60.0};

/// Convert kgmm to Nmm
constexpr double kKilogramMmToNewtonMm{9.80665};

// The initial pose of the robot, in world coordinate system
Pose createInitialPose() {
    return {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}};
}

EncoderCharacteristics createEncoderCharacteristics() {
    EncoderCharacteristics encoder_characteristics{};

    encoder_characteristics.counts_per_revolution = 12.0;
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

MotorPinConfiguration createLeftMotorPinConfiguration() {
    MotorPinConfiguration pin_configuration{};
    AnalogPin pwm_pin{};
    pwm_pin.id = 0;
    DigitalPin in1_pin{};
    in1_pin.id = 1;
    DigitalPin in2_pin{};
    in2_pin.id = 2;
    pin_configuration.pwm_pin = pwm_pin;
    pin_configuration.in1_pin = in1_pin;
    pin_configuration.in2_pin = in2_pin;
    pin_configuration.reverse_direction = false;
    return pin_configuration;
}

MotorPinConfiguration createRightMotorPinConfiguration() {
    MotorPinConfiguration pin_configuration{};
    AnalogPin pwm_pin{};
    pwm_pin.id = 6;
    DigitalPin in1_pin{};
    in1_pin.id = 4;
    DigitalPin in2_pin{};
    in2_pin.id = 5;
    pin_configuration.pwm_pin = pwm_pin;
    pin_configuration.in1_pin = in1_pin;
    pin_configuration.in2_pin = in2_pin;
    pin_configuration.reverse_direction = false;
    return pin_configuration;
}

EncoderPinConfiguration createLeftEncoderPinConfiguration() {
    EncoderPinConfiguration pin_configuration{};
    DigitalPin enc_a_pin{};
    enc_a_pin.id = 3;
    DigitalPin enc_b_pin{};
    enc_b_pin.id = 7;
    pin_configuration.enc_a_pin = enc_a_pin;
    pin_configuration.enc_a_pin = enc_b_pin;
    pin_configuration.reverse_direction = false;
    return pin_configuration;
}

EncoderPinConfiguration createRightEncoderPinConfiguration() {
    EncoderPinConfiguration pin_configuration{};
    DigitalPin enc_a_pin{};
    enc_a_pin.id = 8;
    DigitalPin enc_b_pin{};
    enc_b_pin.id = 9;
    pin_configuration.enc_a_pin = enc_a_pin;
    pin_configuration.enc_a_pin = enc_b_pin;
    pin_configuration.reverse_direction = false;
    return pin_configuration;
}

IrSensorArrayPinConfiguration createIrSensorArrayPinConfiguration() {
    IrSensorArrayPinConfiguration pin_configuration{};
    static_assert(kMaxIrSensorArrayNumberOfLeds == 15U, "Maximum number of leds are not 15!");
    std::array<uint8_t, kMaxIrSensorArrayNumberOfLeds> sensor_pins{
        A9, A8, A7, A6, A5, A4, A3, A2, A1, A0, A17, A16, A15, A14, A13};
    std::array<uint8_t, 2U> emitter_pins{37, 36};
    pin_configuration.sensor_pins = sensor_pins;
    pin_configuration.emitter_pins = emitter_pins;
    return pin_configuration;
}

DifferentialDriveRobotCharacteristics createRobotCharacteristics() {
    DifferentialDriveRobotCharacteristics robot_characteristics{};
    robot_characteristics.wheel_radius = 0.02;
    robot_characteristics.distance_between_wheels = 0.1;
    return robot_characteristics;
}

LineFollowingCharacteristics createLineFollowingCharacteristics() {
    LineFollowingCharacteristics line_following_characteristics{};

    line_following_characteristics.pid_speed_parameters.proportional_gain = 2.0;
    line_following_characteristics.pid_speed_parameters.integral_gain = 0.0;
    line_following_characteristics.pid_speed_parameters.derivative_gain = 0.0;
    line_following_characteristics.pid_speed_parameters.min_value = 0.0;
    line_following_characteristics.pid_speed_parameters.max_value = 2.0;

    line_following_characteristics.pid_steer_parameters.proportional_gain = 2.0;
    line_following_characteristics.pid_steer_parameters.integral_gain = 0.0;
    line_following_characteristics.pid_steer_parameters.derivative_gain = 0.0;
    line_following_characteristics.pid_steer_parameters.min_value = -2.0;
    line_following_characteristics.pid_steer_parameters.max_value = 2.0;

    line_following_characteristics.measurement_noise = 0.1;
    line_following_characteristics.position_state_noise = 0.1;
    line_following_characteristics.position_derivative_state_noise = 0.01;

    line_following_characteristics.max_forward_velocity = 1.0;

    return line_following_characteristics;
}

}  // namespace

class LineFollowerRobot final {
    bool initializeLogging_() {
        LoggingAgent::getInstance().schedule(*scheduler_, kLoggingUpdateRateMicros);
        return true;
    }

 public:
    LineFollowerRobot()
        : scheduler_{std::make_shared<SchedulerProducerAgent>()},
          initialized_logging_{initializeLogging_()},
          left_encoder_data_consumer_agent_{},
          left_encoder_data_producer_agent_{createEncoderCharacteristics(),
                                            createLeftEncoderPinConfiguration(), EncoderTag::kLeft},
          left_motor_signal_consumer_agent_{createMotorCharacteristics(),
                                            createLeftMotorPinConfiguration()},
          right_encoder_data_consumer_agent_{},
          right_encoder_data_producer_agent_{createEncoderCharacteristics(),
                                             createRightEncoderPinConfiguration(),
                                             EncoderTag::kRight},
          right_motor_signal_consumer_agent_{createMotorCharacteristics(),
                                             createRightMotorPinConfiguration()},
          ir_sensor_array_data_producer_agent_{createIrSensorArrayCharacteristics(),
                                               createIrSensorArrayPinConfiguration()},
          line_following_agent_{createRobotCharacteristics(), createLineFollowingCharacteristics(),
                                createInitialPose()} {}

    void setup();
    void loop();

 private:
    std::shared_ptr<SchedulerProducerAgent> scheduler_;
    bool initialized_logging_;
    EncoderDataConsumerAgent left_encoder_data_consumer_agent_;
    EncoderDataProducerAgent left_encoder_data_producer_agent_;
    MotorSignalConsumerAgent left_motor_signal_consumer_agent_;
    EncoderDataConsumerAgent right_encoder_data_consumer_agent_;
    EncoderDataProducerAgent right_encoder_data_producer_agent_;
    MotorSignalConsumerAgent right_motor_signal_consumer_agent_;
    IrSensorArrayDataProducerAgent ir_sensor_array_data_producer_agent_;
    LineFollowingAgent line_following_agent_;
};

void LineFollowerRobot::setup() {
    left_encoder_data_consumer_agent_.onReceiveData([this](EncoderData const& encoder_data) {
        LOG_INFO("Left encoder data: %.2f rev/s", encoder_data.revolutions_per_second);
        line_following_agent_.setEncoderLeftData(encoder_data);
    });

    right_encoder_data_consumer_agent_.onReceiveData([this](EncoderData const& encoder_data) {
        LOG_INFO("Right encoder data: %.2f rev/s", encoder_data.revolutions_per_second);
        line_following_agent_.setEncoderRightData(encoder_data);
    });

    line_following_agent_.onReceiveData([this](IrSensorArrayData const& ir_sensor_array_data) {
        Pose pose{line_following_agent_.getPose()};

        line_following_agent_.setIrSensorArrayData(ir_sensor_array_data);
        line_following_agent_.step();

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
    left_encoder_data_consumer_agent_.attach(left_encoder_data_producer_agent_);
    right_encoder_data_consumer_agent_.attach(right_encoder_data_producer_agent_);
    line_following_agent_.attach(ir_sensor_array_data_producer_agent_);

    // Attach motors to line follower agent
    line_following_agent_.attachMotorLeft(left_motor_signal_consumer_agent_);
    line_following_agent_.attachMotorRight(right_motor_signal_consumer_agent_);

    LOG_INFO("Starting calibration routine for ir sensor...", "");
    ir_sensor_array_data_producer_agent_.calibrate(kCalibrationIterations);

    // Start scheduling readings from sensors
    left_encoder_data_producer_agent_.schedule(*scheduler_, kUpdateRateMicros);
    right_encoder_data_producer_agent_.schedule(*scheduler_, kUpdateRateMicros);
    ir_sensor_array_data_producer_agent_.schedule(*scheduler_, kUpdateRateMicros);

    LOG_INFO("I am alive!", "");
}

void LineFollowerRobot::loop() {
    // Step the software using the scheduler
    scheduler_->tick();
}

}  // namespace line_follower

void setup() {
    Serial.begin(115200);
}

void loop() {
    line_follower::LineFollowerRobot robot{};
    robot.setup();
    while (true) robot.loop();
}