// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_GEOMETRY_CONVERSION_H_
#define LINE_FOLLOWER_BLOCKS_GEOMETRY_CONVERSION_H_

#include <array>

#include "line_follower/blocks/geometry/vector.h"
#include "line_follower/blocks/geometry/quaternion.h"
#include "line_follower/blocks/geometry/line.h"
#include "line_follower/types/position.h"
#include "line_follower/types/rotation.h"
#include "line_follower/types/line.h"

namespace line_follower
{

/// Convert a Position to a geometry::Vector3
inline geometry::Vector3<double> convert(Position const& position)
{
    return geometry::Vector3<double>{position.x, position.y, position.z};
}

/// Convert a geometry::Vector3 to a Position
template <typename T>
inline Position convert(geometry::Vector3<T> const& vector)
{
    return Position{vector[0], vector[1], vector[2]};
}

/// Convert an EulerRotation to a geometry::EulerAngles
inline geometry::EulerAngles<double> convert(EulerRotation const& a)
{
    return {a.roll, a.pitch, a.yaw};
}

/// Convert an geometry::EulerAngles to a EulerRotation
template <typename T>
inline EulerRotation convert(geometry::EulerAngles<T> const& a)
{
    return {a.roll(), a.pitch(), a.yaw()};
}


/// Convert a QuaternionRotation to a geometry::Quaternion
inline geometry::Quaternion<double> convert(QuaternionRotation const& q)
{
    auto output{geometry::Quaternion<double>{q.w, q.x, q.y, q.z}};

    if (output.is_zero())
    {
        return geometry::Quaternion<double>{1.0, 0.0, 0.0, 0.0};
    }

    return output;
}

/// Convert a geometry::Quaternion to a QuaternionRotation
template <typename T>
inline QuaternionRotation convert(geometry::Quaternion<T> const& q)
{
    return QuaternionRotation(q.w(), q.x(), q.y(), q.z());
}

/// Convert a EulerRotation to ageometry:: Quaternion
inline geometry::Quaternion<double> eulerToQuat(EulerRotation const& a)
{
    return from_euler(convert(a));
}

/// Convert a geometry::Quaternion to a EulerRotation
template <typename T>
inline EulerRotation quatToEuler(geometry::Quaternion<T> const& q)
{
    return convert(to_euler(q));
}

/// Convert a geometry::Line to a Line
template <typename T>
inline Line convert(geometry::Line<T> const& line)
{
    return Line{line.from(), line.to()};
}

/// Convert a Line to a geometry::Line
inline geometry::Line<double> convert(Line const& line)
{
    return geometry::Line<double>{convert(line.start), convert(line.end)};
}

}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_GEOMETRY_CONVERSION_H_