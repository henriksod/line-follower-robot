// Copyright (c) 2024 Henrik Söderlund

#include <Arduino.h>
#include <LineFollower.h>

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

}  // namespace

class LineFollowerRobot final
{
 public:
    LineFollowerRobot()
        : scheduler_{std::make_shared<SchedulerProducerAgent>()},
          left_encoder_data_consumer_agent_{},
          left_encoder_data_producer_agent_{createEncoderCharacteristics(), createLeftEncoderPinConfiguration(), EncoderTag::kLeft},
          left_motor_signal_producer_agent_{},
          left_motor_signal_consumer_agent_{createMotorCharacteristics(), createLeftMotorPinConfiguration()}
    {
        LoggingAgent::getInstance().schedule(scheduler_, kLoggingUpdateRateMicros);
    }

    void setup();
    void loop();
 private:
    std::shared_ptr<SchedulerProducerAgent> scheduler_;
    EncoderDataConsumerAgent left_encoder_data_consumer_agent_;
    EncoderDataProducerAgent left_encoder_data_producer_agent_;
    MotorSignalProducerAgent left_motor_signal_producer_agent_;
    MotorSignalConsumerAgent left_motor_signal_consumer_agent_;
};

void LineFollowerRobot::setup() {
    left_encoder_data_consumer_agent_.onReceiveData([this](EncoderData const& encoder_data) {
        LOG_INFO("Left encoder data: %.2f rev/s", encoder_data.revolutions_per_second);
    });

    left_encoder_data_consumer_agent_.attach(left_encoder_data_producer_agent_);
    left_motor_signal_consumer_agent_.attach(left_motor_signal_producer_agent_);

    MotorSignal signal{};
    signal.speed.revolutions_per_second = 1.0;
    left_motor_signal_producer_agent_.sendData(signal);

    LOG_INFO("I am alive!", "");
}

void LineFollowerRobot::loop() {
    // Step the software using the scheduler
    scheduler_->tick();
}

}  // namespace line_follower

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    line_follower::LineFollowerRobot robot{};
    robot.setup();
    while (true) robot.loop();
}