// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_GEOMETRY_EULER_ANGLES_H_
#define LINE_FOLLOWER_BLOCKS_GEOMETRY_EULER_ANGLES_H_

#include <cmath>
#include <iterator>
#include <limits>
#include <type_traits>

#include "line_follower/blocks/common/math.h"

namespace line_follower {
namespace geometry {
/// @brief An EulerAngles class
/// @tparam T Any numeric type
template <class T>
class EulerAngles {
 public:
    typedef T value_type;

    static_assert(std::is_same<T, float>() || std::is_same<T, double>(),
                  "Invalid scalar type for EulerAngles");

    EulerAngles() : roll_{0.0}, pitch_{0.0}, yaw_{0.0} {}

    EulerAngles(T roll, T pitch, T yaw) : roll_{roll}, pitch_{pitch}, yaw_{yaw} {}

    template <typename T1>
    explicit EulerAngles(const EulerAngles<T1>& other)
        : roll_{static_cast<T>(other.roll())},
          pitch_{static_cast<T>(other.pitch())},
          yaw_{static_cast<T>(other.yaw())} {}

    template <typename T1>
    EulerAngles& operator=(const EulerAngles<T1>& other) {
        roll_ = other.roll();
        pitch_ = other.pitch();
        yaw_ = other.yaw();
        return *this;
    }

    /// Roll in radians
    T roll() const { return roll_; }

    /// Pitch in radians
    T pitch() const { return pitch_; }

    /// Yaw in radians
    T yaw() const { return yaw_; }

    template <typename T1 = T>
    bool is_zero(T1 eps = 0) const {
        return is_scalar_zero(roll_, eps) && is_scalar_zero(pitch_, eps) &&
               is_scalar_zero(yaw_, eps);
    }

    bool is_nan() const { return std::isnan(roll_) || std::isnan(pitch_) || std::isnan(yaw_); }

    bool is_inf() const { return std::isinf(roll_) || std::isinf(pitch_) || std::isinf(yaw_); }

    bool is_finite() const {
        return std::isfinite(roll_) && std::isfinite(pitch_) && std::isfinite(yaw_);
    }

    EulerAngles operator+() const { return *this; }

    EulerAngles operator-() const { return {-roll_, -pitch_, -yaw_}; }

    EulerAngles operator+=(T scalar) {
        roll_ += scalar;
        pitch_ += scalar;
        yaw_ += scalar;
        return *this;
    }

    EulerAngles operator-=(T scalar) {
        roll_ -= scalar;
        pitch_ -= scalar;
        yaw_ -= scalar;
        return *this;
    }

    EulerAngles operator*=(T k) {
        roll_ = k * roll_;
        pitch_ = k * pitch_;
        yaw_ = k * yaw_;
        return *this;
    }

    EulerAngles operator/=(T k) {
        roll_ /= k;
        pitch_ /= k;
        yaw_ /= k;
        return *this;
    }

    template <typename T1>
    EulerAngles operator+=(EulerAngles<T1> const& other) {
        roll_ += other.roll();
        pitch_ += other.pitch();
        yaw_ += other.yaw();
        return *this;
    }

    template <typename T1>
    EulerAngles operator-=(EulerAngles<T1> const& other) {
        roll_ -= other.roll();
        pitch_ -= other.pitch();
        yaw_ -= other.yaw();
        return *this;
    }

    template <typename T1>
    EulerAngles operator*=(EulerAngles<T1> const& other) {
        roll_ *= other.roll();
        pitch_ *= other.pitch();
        yaw_ *= other.yaw();
        return *this;
    }

    template <typename T1>
    EulerAngles operator/=(const EulerAngles<T1>& other) {
        roll_ /= other.roll();
        pitch_ /= other.pitch();
        yaw_ /= other.yaw();
        return *this;
    }

 private:
    T roll_, pitch_, yaw_;  // the full state for a EulerAngles
};
}  // namespace geometry
}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_GEOMETRY_EULER_ANGLES_H_
