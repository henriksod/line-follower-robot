// Copyright (c) 2024 Henrik Söderlund

#include "line_follower/external/api/event_logger_agent.h"

#include <fstream>
#include <string>

#include "line_follower/external/api/common.h"
#include "line_follower/external/api/logging.h"
#include "line_follower/external/types/encoder_data.h"
#include "line_follower/external/types/ir_sensor_array_data.h"
#include "line_follower/external/types/line_follower_event_state.h"
#include "line_follower/external/types/position.h"
#include "line_follower/external/types/rotation.h"
#include "line_follower/external/types/system_time.h"
#include "nlohmann/json.hpp"

namespace line_follower {

// Conversion functions for SystemTime
void to_json(nlohmann::json& j, const SystemTime& t) {
    j = {{"system_time_us", t.system_time_us}};
}

void from_json(const nlohmann::json& j, SystemTime& t) {
    j.at("system_time_us").get_to(t.system_time_us);
}

// Conversion functions for EncoderData
void to_json(nlohmann::json& j, const EncoderData& e) {
    j = {{"revolutions_per_second", e.revolutions_per_second}, {"step_position", e.step_position}};
}

void from_json(const nlohmann::json& j, EncoderData& e) {
    j.at("revolutions_per_second").get_to(e.revolutions_per_second);
    j.at("step_position").get_to(e.step_position);
}

// Conversion functions for Position
void to_json(nlohmann::json& j, const Position& p) {
    j = {{"x", p.x}, {"y", p.y}, {"z", p.z}};
}

void from_json(const nlohmann::json& j, Position& p) {
    j.at("x").get_to(p.x);
    j.at("y").get_to(p.y);
    j.at("z").get_to(p.z);
}

// Conversion functions for QuaternionRotation
void to_json(nlohmann::json& j, const QuaternionRotation& q) {
    j = {{"w", q.w}, {"x", q.x}, {"y", q.y}, {"z", q.z}};
}

void from_json(const nlohmann::json& j, QuaternionRotation& q) {
    j.at("w").get_to(q.w);
    j.at("x").get_to(q.x);
    j.at("y").get_to(q.y);
    j.at("z").get_to(q.z);
}

// Conversion functions for Pose
void to_json(nlohmann::json& j, const Pose& p) {
    j = {{"position", p.position}, {"rotation", p.rotation}};
}

void from_json(const nlohmann::json& j, Pose& p) {
    j.at("position").get_to(p.position);
    j.at("rotation").get_to(p.rotation);
}

// Conversion functions for LineFollowerEventState
void to_json(nlohmann::json& j, const LineFollowerEventState& l) {
    j = {{"timestamp", l.timestamp},
         {"left_encoder_data_input", l.left_encoder_data_input},
         {"right_encoder_data_input", l.right_encoder_data_input},
         {"global_pose", l.global_pose}};
}

void from_json(const nlohmann::json& j, LineFollowerEventState& l) {
    j.at("timestamp").get_to(l.timestamp);
    j.at("left_encoder_data_input").get_to(l.left_encoder_data_input);
    j.at("right_encoder_data_input").get_to(l.right_encoder_data_input);
    j.at("global_pose").get_to(l.global_pose);
}

template <>
void EventLoggerAgent<IrSensorArrayData, LineFollowerEventState>::logEvent(
    std::string const& filename, LineFollowerEventState const event_state) {
    std::ifstream infile(filename);
    nlohmann::json json_data;

    // Read existing content of the file if it exists
    if (infile.is_open()) {
        infile >> json_data;
        infile.close();
    }

    // If file is empty or doesn't contain an array, initialize it as an array
    if (!json_data.is_array()) {
        json_data = nlohmann::json::array();
    }

    // Append the new event
    json_data.push_back(event_state);

    // Write the updated data back to the file
    std::ofstream outfile(filename);
    if (outfile.is_open()) {
        outfile << json_data.dump(4);  // Pretty print with an indentation of 4 spaces
        outfile.close();
    } else {
        LOG_ERROR("Error opening the file for writing!");
    }
}

}  // namespace line_follower
