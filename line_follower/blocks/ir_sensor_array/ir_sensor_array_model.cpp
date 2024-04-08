// Copyright (c) 2023 Henrik Söderlund

#include "line_follower/blocks/ir_sensor_array/ir_sensor_array_model.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <vector>

#include "line_follower/blocks/common/math.h"
#include "line_follower/blocks/geometry/conversion.h"
#include "line_follower/blocks/geometry/line.h"
#include "line_follower/blocks/geometry/quaternion.h"
#include "line_follower/blocks/geometry/utils/line_utils.h"
#include "line_follower/blocks/geometry/utils/rotation_utils.h"
#include "line_follower/blocks/geometry/vector.h"

namespace line_follower {
namespace {
/// Millimeters to Meters
constexpr double kMillimetersToMeters{0.001};
}  // namespace

void IrSensorArrayModel::tick(SystemTime const timestamp) {
    ir_sensor_array_data_.timestamp = timestamp;
}

bool IrSensorArrayModel::getIrSensorArrayData(IrSensorArrayData& output) const {
    static_cast<void>(memcpy(output.ir_sensor_readings.begin(),
                             ir_sensor_array_data_.ir_sensor_readings.begin(),
                             output.ir_sensor_readings.size() * sizeof(IrSensorData)));
    output.valid = ir_sensor_array_data_.valid;
    output.timestamp = ir_sensor_array_data_.timestamp;
    output.number_of_leds = ir_sensor_array_data_.number_of_leds;
    return true;
}

void IrSensorArrayModel::setTrackLines(TrackSegment const& track_segment,
                                       Pose const& ir_sensor_array_pose_in_track_segment) {
    using geometry::Line;
    using geometry::Vector3;
    resetIrSensorArrayData(ir_sensor_array_data_);
    ir_sensor_array_data_.number_of_leds = ir_array_characteristics_.number_of_leds;

    static_cast<void>(memcpy(&current_track_segment_, &track_segment, sizeof(TrackSegment)));

    Vector3<double> ir_sensor_array_position{
        convert(ir_sensor_array_pose_in_track_segment.position)};

    for (std::size_t idx{0U}; idx < ir_array_characteristics_.number_of_leds; ++idx) {
        double led_idx{static_cast<double>(idx)};
        double middle_led_idx{std::floor(ir_array_characteristics_.number_of_leds / 2.0)};
        double array_spacing{ir_array_characteristics_.array_spacing * kMillimetersToMeters};

        // Get the led position relative to the origin, which is the middle of the
        // ir array
        Vector3<double> led_position_rel_origin{(led_idx - middle_led_idx) * array_spacing, 0.0,
                                                0.0};

        // Convert the point from the previous led to this led into a line
        Line<double> led_line{};
        led_line.set(ir_sensor_array_position +
                         rotated(convert(ir_sensor_array_pose_in_track_segment.rotation),
                                 Vector3<double>{led_position_rel_origin[0] - array_spacing / 2.0,
                                                 0.0, 0.0}),
                     ir_sensor_array_position +
                         rotated(convert(ir_sensor_array_pose_in_track_segment.rotation),
                                 Vector3<double>{led_position_rel_origin[0] + array_spacing / 2.0,
                                                 0.0, 0.0}));

        Vector3<double> led_line_vector{normalized(led_line.to() - led_line.from())};

        /// Check for overlaps
        for (auto& track_line_segment : track_segment.track_lines) {
            if (!track_line_segment.visible) {
                continue;
            }

            auto orthogonal_led_line_vector{
                rotated(geometry::detail::k90DegreesAroundZ, led_line_vector)};
            Line<double> orthogonal_led_line{
                led_line.center() + orthogonal_led_line_vector * array_spacing / 2.0,
                led_line.center() - orthogonal_led_line_vector * array_spacing / 2.0};

            Line<double> const track_line{convert(track_line_segment.line)};
            auto const track_line_strip{
                geometry::sweepAlongWidth(track_line, track_line_segment.width)};
            bool any_overlap{led_line.intersectsWithAny(track_line_strip) ||
                             orthogonal_led_line.intersectsWithAny(track_line_strip)};

            auto& led{ir_sensor_array_data_.ir_sensor_readings.at(idx)};
            led.detected_white_surface = true;
            led.intensity = 1.0;

            if (any_overlap) {
                ir_sensor_array_data_.valid = true;
                led.detected_white_surface = false;

                /// TODO: Calculate digital reading based on distance from track line
                // edge wrt thickness
                led.intensity = track_line_segment.whiteness;
            }
        }
    }
}  // IrSensorArrayModel::setTrackLines
}  // namespace line_follower
