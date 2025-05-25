// Copyright (c) 2025 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_UTILITIES_JSON_CONVERSION_H_
#define LINE_FOLLOWER_BLOCKS_UTILITIES_JSON_CONVERSION_H_

#include "line_follower/external/types/encoder_data.h"
#include "line_follower/external/types/ir_sensor_array_data.h"
#include "line_follower/external/types/line_follower_event_state.h"
#include "line_follower/external/types/line_following_statistics.h"
#include "line_follower/external/types/pose.h"
#include "line_follower/external/types/position.h"
#include "line_follower/external/types/rotation.h"
#include "line_follower/external/types/system_time.h"
#include "nlohmann/json.hpp"

/// TODO: Autogenerate this code using a script

// Conversion functions for SystemTime
void to_json(nlohmann::json& j, const line_follower::SystemTime& t) {
    j = {{"system_time_us", t.system_time_us}};
}

void from_json(const nlohmann::json& j, line_follower::SystemTime& t) {
    j.at("system_time_us").get_to(t.system_time_us);
}

// Conversion functions for EncoderData
void to_json(nlohmann::json& j, const line_follower::EncoderData& e) {
    j = {{"revolutions_per_second", e.revolutions_per_second}, {"step_position", e.step_position}};
}

void from_json(const nlohmann::json& j, line_follower::EncoderData& e) {
    j.at("revolutions_per_second").get_to(e.revolutions_per_second);
    j.at("step_position").get_to(e.step_position);
}

// Conversion functions for LineFollowingStatistics
void to_json(nlohmann::json& j, const line_follower::LineFollowingStatistics& t) {
    j = {{"tracking_error", t.tracking_error},
         {"average_speed", t.average_speed},
         {"time_spent_on_line", t.time_spent_on_line}};
}

void from_json(const nlohmann::json& j, line_follower::LineFollowingStatistics& t) {
    j.at("tracking_error").get_to(t.tracking_error);
    j.at("average_speed").get_to(t.average_speed);
    j.at("time_spent_on_line").get_to(t.time_spent_on_line);
}

// Conversion functions for Position
void to_json(nlohmann::json& j, const line_follower::Position& p) {
    j = {{"x", p.x}, {"y", p.y}, {"z", p.z}};
}

void from_json(const nlohmann::json& j, line_follower::Position& p) {
    j.at("x").get_to(p.x);
    j.at("y").get_to(p.y);
    j.at("z").get_to(p.z);
}

// Conversion functions for QuaternionRotation
void to_json(nlohmann::json& j, const line_follower::QuaternionRotation& q) {
    j = {{"w", q.w}, {"x", q.x}, {"y", q.y}, {"z", q.z}};
}

void from_json(const nlohmann::json& j, line_follower::QuaternionRotation& q) {
    j.at("w").get_to(q.w);
    j.at("x").get_to(q.x);
    j.at("y").get_to(q.y);
    j.at("z").get_to(q.z);
}

// Conversion functions for Pose
void to_json(nlohmann::json& j, const line_follower::Pose& p) {
    nlohmann::json position_json;
    nlohmann::json rotation_json;
    to_json(position_json, p.position);
    to_json(rotation_json, p.rotation);
    j = {{"position", position_json}, {"rotation", rotation_json}};
}

void from_json(const nlohmann::json& j, line_follower::Pose& p) {
    from_json(j.at("position"), p.position);
    from_json(j.at("rotation"), p.rotation);
}

// Conversion functions for LineFollowerEventState
void to_json(nlohmann::json& j, const line_follower::LineFollowerEventState& l) {
    nlohmann::json timestamp_json;
    nlohmann::json left_encoder_data_input_json;
    nlohmann::json right_encoder_data_input_json;
    nlohmann::json global_pose_json;
    nlohmann::json ir_pose_json;
    nlohmann::json line_following_statistics_json;
    to_json(timestamp_json, l.timestamp);
    to_json(left_encoder_data_input_json, l.left_encoder_data_input);
    to_json(right_encoder_data_input_json, l.right_encoder_data_input);
    to_json(global_pose_json, l.global_pose);
    to_json(ir_pose_json, l.ir_pose);
    to_json(line_following_statistics_json, l.line_following_statistics);
    j = {{"timestamp", timestamp_json},
         {"left_encoder_data_input", left_encoder_data_input_json},
         {"right_encoder_data_input", right_encoder_data_input_json},
         {"global_pose", global_pose_json},
         {"ir_pose", ir_pose_json},
         {"line_following_statistics", line_following_statistics_json}};
}

void from_json(const nlohmann::json& j, line_follower::LineFollowerEventState& l) {
    from_json(j.at("timestamp"), l.timestamp);
    from_json(j.at("left_encoder_data_input"), l.left_encoder_data_input);
    from_json(j.at("right_encoder_data_input"), l.right_encoder_data_input);
    from_json(j.at("global_pose"), l.global_pose);
    from_json(j.at("ir_pose"), l.ir_pose);
    from_json(j.at("line_following_statistics"), l.line_following_statistics);
}

#endif  // LINE_FOLLOWER_BLOCKS_UTILITIES_JSON_CONVERSION_H_
