// Copyright (c) 2025 Henrik Söderlund

#include "line_follower/blocks/utilities/calibration_loader.h"

#include <fstream>

#include "line_follower/external/api/logging.h"
#include "line_follower/external/types/line_following_characteristics.h"
#include "line_follower/external/types/pid_configuration.h"
#include "nlohmann/json.hpp"

namespace line_follower {
namespace calibration_loader {

namespace {

bool loadPidConfigurationFromJson(nlohmann::json const& data, PIDConfiguration& pid_configuration) {
    // Validate and assign pid values
    if (data.contains("proportional_gain") && data["proportional_gain"].is_number()) {
        pid_configuration.proportional_gain = data["proportional_gain"];
    } else {
        LOG_ERROR("Invalid or missing 'proportional_gain'.");
        return false;
    }

    if (data.contains("integral_gain") && data["integral_gain"].is_number()) {
        pid_configuration.integral_gain = data["integral_gain"];
    } else {
        LOG_ERROR("Invalid or missing 'integral_gain'.");
        return false;
    }

    if (data.contains("derivative_gain") && data["derivative_gain"].is_number()) {
        pid_configuration.derivative_gain = data["derivative_gain"];
    } else {
        LOG_ERROR("Invalid or missing 'derivative_gain'.");
        return false;
    }

    if (data.contains("max_value") && data["max_value"].is_number()) {
        pid_configuration.max_value = data["max_value"];
    } else {
        LOG_ERROR("Invalid or missing 'max_value'.");
        return false;
    }

    if (data.contains("min_value") && data["min_value"].is_number()) {
        pid_configuration.min_value = data["min_value"];
    } else {
        LOG_ERROR("Invalid or missing 'min_value'.");
        return false;
    }

    return true;
}

}  // namespace

bool loadCalibrationFromJson(std::string const& calibration_file,
                             LineFollowingCharacteristics& calibration) {
    std::ifstream f{calibration_file};
    if (!f.is_open()) {
        LOG_ERROR("Failed to open JSON file at path: " + calibration_file);
        return false;
    }

    nlohmann::json data;
    f >> data;

    if (f.fail()) {
        LOG_ERROR("Failed to parse JSON file at path: " + calibration_file);
        return false;
    }

    if (!data.is_object()) {
        LOG_ERROR("Invalid JSON structure: Top level of json is not an object.");
        return false;
    }

    // Load PID speed parameters
    if (data.contains("pid_speed_parameters") && data["pid_speed_parameters"].is_object()) {
        const auto& pid_speed_params = data["pid_speed_parameters"];
        if (!loadPidConfigurationFromJson(pid_speed_params, calibration.pid_speed_parameters)) {
            LOG_ERROR("Invalid or missing 'pid_speed_parameters'.");
            return false;
        }
    } else {
        LOG_ERROR("Invalid or missing 'pid_speed_parameters'.");
        return false;
    }

    // Load PID steer parameters
    if (data.contains("pid_steer_parameters") && data["pid_steer_parameters"].is_object()) {
        const auto& pid_steer_params = data["pid_steer_parameters"];
        if (!loadPidConfigurationFromJson(pid_steer_params, calibration.pid_steer_parameters)) {
            LOG_ERROR("Invalid or missing 'pid_steer_parameters'.");
            return false;
        }
    } else {
        LOG_ERROR("Invalid or missing 'pid_steer_parameters'.");
        return false;
    }

    if (data.contains("max_forward_velocity") && data["max_forward_velocity"].is_number()) {
        calibration.max_forward_velocity = data["max_forward_velocity"];
    } else {
        LOG_ERROR("Invalid or missing 'max_forward_velocity'.");
        return false;
    }

    if (data.contains("turning_speed_ratio") && data["turning_speed_ratio"].is_number()) {
        calibration.turning_speed_ratio = data["turning_speed_ratio"];
    } else {
        LOG_ERROR("Invalid or missing 'turning_speed_ratio'.");
        return false;
    }

    if (data.contains("sharp_turn_forward_velocity") &&
        data["sharp_turn_forward_velocity"].is_number()) {
        calibration.sharp_turn_forward_velocity = data["sharp_turn_forward_velocity"];
    } else {
        LOG_ERROR("Invalid or missing 'sharp_turn_forward_velocity'.");
        return false;
    }

    if (data.contains("sharp_turn_angular_velocity") &&
        data["sharp_turn_angular_velocity"].is_number()) {
        calibration.sharp_turn_angular_velocity = data["sharp_turn_angular_velocity"];
    } else {
        LOG_ERROR("Invalid or missing 'sharp_turn_angular_velocity'.");
        return false;
    }

    return true;
}

}  // namespace calibration_loader
}  // namespace line_follower
