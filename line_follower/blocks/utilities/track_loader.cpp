// Copyright (c) 2024 Henrik Söderlund

#include "line_follower/blocks/utilities/track_loader.h"

#include <cstdint>
#include <fstream>
#include <vector>

#include "line_follower/external/api/logging.h"
#include "line_follower/external/types/pose.h"
#include "line_follower/external/types/position.h"
#include "line_follower/external/types/rotation.h"
#include "line_follower/external/types/track_lines.h"
#include "nlohmann/json.hpp"

namespace line_follower {
namespace track_loader {

namespace {

bool loadPositionFromJson(nlohmann::json const& data, Position& position) {
    // Validate and assign position values
    if (data.contains("x") && data["x"].is_number()) {
        position.x = data["x"];
    } else {
        LOG_ERROR("Invalid or missing 'x' in 'position'.");
        return false;
    }

    if (data.contains("y") && data["y"].is_number()) {
        position.y = data["y"];
    } else {
        LOG_ERROR("Invalid or missing 'y' in 'position'.");
        return false;
    }

    if (data.contains("z") && data["z"].is_number()) {
        position.z = data["z"];
    } else {
        LOG_ERROR("Invalid or missing 'z' in 'position'.");
        return false;
    }

    return true;
}

bool loadRotationFromJson(nlohmann::json const& data, QuaternionRotation& rotation) {
    // Validate and assign rotation values
    if (data.contains("w") && data["w"].is_number()) {
        rotation.w = data["w"];
    } else {
        LOG_ERROR("Invalid or missing 'w' in 'rotation'.");
        return false;
    }

    if (data.contains("x") && data["x"].is_number()) {
        rotation.x = data["x"];
    } else {
        LOG_ERROR("Invalid or missing 'x' in 'rotation'.");
        return false;
    }

    if (data.contains("y") && data["y"].is_number()) {
        rotation.y = data["y"];
    } else {
        LOG_ERROR("Invalid or missing 'y' in 'rotation'.");
        return false;
    }

    if (data.contains("z") && data["z"].is_number()) {
        rotation.z = data["z"];
    } else {
        LOG_ERROR("Invalid or missing 'z' in 'rotation'.");
        return false;
    }

    return true;
}

}  // namespace

bool loadScenarioFromJson(const std::string& path,
                          std::vector<line_follower::TrackSegment>& track_segments,
                          Pose& robot_initial_pose) {
    std::ifstream f{path};
    if (!f.is_open()) {
        LOG_ERROR("Failed to open JSON file at path: " + path);
        return false;
    }

    nlohmann::json data;
    f >> data;

    if (f.fail()) {
        LOG_ERROR("Failed to parse JSON file at path: " + path);
        return false;
    }

    if (!data.is_object()) {
        LOG_ERROR("Invalid JSON structure: Top level of json is not an object.");
        return false;
    }

    if (!data.contains("robot_initial_pose") || !data["robot_initial_pose"].is_object()) {
        LOG_ERROR("Invalid JSON structure: 'robot_initial_pose' not found or not an object.");
        return false;
    }

    const auto& robot_initial_pose_json = data["robot_initial_pose"];
    const auto& robot_initial_position_json = robot_initial_pose_json["position"];
    const auto& robot_initial_rotation_json = robot_initial_pose_json["rotation"];

    if (!loadPositionFromJson(robot_initial_position_json, robot_initial_pose.position)) {
        return false;
    }

    if (!loadRotationFromJson(robot_initial_rotation_json, robot_initial_pose.rotation)) {
        return false;
    }

    if (!data.contains("track_segments") || !data["track_segments"].is_array()) {
        LOG_ERROR("Invalid JSON structure: 'track_segments' not found or not an array.");
        return false;
    }

    for (const auto& track_segment_json : data["track_segments"]) {
        if (!track_segment_json.is_object() || !track_segment_json.contains("pose") ||
            !track_segment_json["pose"].is_object()) {
            LOG_ERROR("Invalid JSON structure: 'pose' not found or not an object.");
            return false;
        }

        if (!track_segment_json.contains("track_lines") ||
            !track_segment_json["track_lines"].is_array()) {
            LOG_ERROR("Invalid JSON structure: 'track_lines' not found or not an array.");
            return false;
        }

        line_follower::TrackSegment track_segment;

        const auto& pose_json = track_segment_json["pose"];
        const auto& position_json = pose_json["position"];
        const auto& rotation_json = pose_json["rotation"];

        if (!position_json.is_object() || !rotation_json.is_object()) {
            LOG_ERROR("Invalid JSON structure: 'position' or 'rotation' is not an object.");
            return false;
        }

        if (!loadPositionFromJson(position_json, track_segment.pose.position)) {
            return false;
        }

        if (!loadRotationFromJson(rotation_json, track_segment.pose.rotation)) {
            return false;
        }

        std::size_t num_track_lines = 0;
        for (const auto& track_line_json : track_segment_json["track_lines"]) {
            if (num_track_lines >= line_follower::kMaxNumberOfLinesPerTrackSegment) {
                LOG_ERROR("Number of track lines exceeds the maximum per track segment.");
                break;
            }

            line_follower::TrackLineSegment track_line;

            if (track_line_json.contains("start") && track_line_json["start"].is_object()) {
                const auto& start_json = track_line_json["start"];
                if (!loadPositionFromJson(start_json, track_line.line.start)) {
                    return false;
                }
            } else {
                LOG_ERROR("Invalid or missing 'start' in track line.");
                return false;
            }

            if (track_line_json.contains("end") && track_line_json["end"].is_object()) {
                const auto& end_json = track_line_json["end"];
                if (!loadPositionFromJson(end_json, track_line.line.end)) {
                    return false;
                }
            } else {
                LOG_ERROR("Invalid or missing 'end' in track line.");
                return false;
            }

            if (track_line_json.contains("width") && track_line_json["width"].is_number()) {
                track_line.width = track_line_json["width"];
            } else {
                LOG_ERROR("Invalid or missing 'width' in track line.");
                return false;
            }

            if (track_line_json.contains("whiteness") && track_line_json["whiteness"].is_number()) {
                track_line.whiteness = track_line_json["whiteness"];
            } else {
                LOG_ERROR("Invalid or missing 'whiteness' in track line.");
                return false;
            }

            if (track_line_json.contains("visible") && track_line_json["visible"].is_boolean()) {
                track_line.visible = track_line_json["visible"];
            } else {
                LOG_ERROR("Invalid or missing 'visible' in track line.");
                return false;
            }

            track_segment.track_lines.at(num_track_lines) = track_line;
            ++num_track_lines;
        }

        track_segments.push_back(track_segment);
    }

    return true;
}

}  // namespace track_loader
}  // namespace line_follower
