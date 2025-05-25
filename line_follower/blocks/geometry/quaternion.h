// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_GEOMETRY_QUATERNION_H_
#define LINE_FOLLOWER_BLOCKS_GEOMETRY_QUATERNION_H_

#include <algorithm>
#include <cmath>  // for atan2, which handles signs for us properly
#include <iterator>
#include <limits>
#include <type_traits>

#include "line_follower/blocks/common/math.h"
#include "line_follower/blocks/common/maybe.h"
#include "line_follower/blocks/geometry/euler_angles.h"
#include "line_follower/blocks/geometry/vector.h"

namespace line_follower {
namespace geometry {
/// @brief A Quaternion class
/// @tparam T Any numeric type
template <class T>
class Quaternion {
 public:
    typedef T value_type;

    static_assert(std::is_same<T, float>() || std::is_same<T, double>(),
                  "Invalid scalar type for Quaternion");

    Quaternion() : w_{1.0}, x_{0.0}, y_{0.0}, z_{0.0} {}

    Quaternion(T w, T x, T y, T z) : w_{w}, x_{x}, y_{y}, z_{z} {}

    template <typename T1>
    explicit Quaternion(const Quaternion<T1>& other)
        : w_{other.w()}, x_{other.x()}, y_{other.y()}, z_{other.z()} {}

    template <typename T1>
    explicit Quaternion(const Vector3<T1>& vector)
        : w_{static_cast<T1>(0.0)}, x_{vector.x()}, y_{vector.y()}, z_{vector.z()} {}

    template <typename T1>
    Quaternion& operator=(const Quaternion<T1>& other) {
        w_ = other.w();
        x_ = other.x();
        y_ = other.y();
        z_ = other.z();
        return *this;
    }

    T w() const { return w_; }

    T x() const { return x_; }

    T y() const { return y_; }

    T z() const { return z_; }

    T real() const { return w_; }

    Quaternion complex() const { return {0, x_, y_, z_}; }

    T norm_squared() const { return w_ * w_ + x_ * x_ + y_ * y_ + z_ * z_; }

    T norm() const { return std::sqrt(norm_squared()); }

    T complex_norm_squared() const { return x_ * x_ + y_ * y_ + z_ * z_; }

    template <typename T1 = T>
    bool is_zero(T1 eps = 0) const {
        return is_scalar_zero(w_, eps) && is_scalar_zero(x_, eps) && is_scalar_zero(y_, eps) &&
               is_scalar_zero(z_, eps);
    }

    template <typename T1 = T>
    bool is_real(T1 eps = 0) const {
        return !is_scalar_zero(w_, eps) && is_scalar_zero(x_, eps) && is_scalar_zero(y_, eps) &&
               is_scalar_zero(z_, eps);
    }

    template <typename T1 = T>
    bool is_complex(T1 eps = 0) const {
        return !is_real(eps);
    }

    bool is_nan() const {
        return std::isnan(w_) || std::isnan(x_) || std::isnan(y_) || std::isnan(z_);
    }

    bool is_inf() const {
        return std::isinf(w_) || std::isinf(x_) || std::isinf(y_) || std::isinf(z_);
    }

    bool is_finite() const {
        return std::isfinite(w_) && std::isfinite(x_) && std::isfinite(y_) && std::isfinite(z_);
    }

    template <typename T1 = T>
    bool is_unit(T1 eps = 0) const {
        return is_scalar_zero(norm_squared() - T(1), eps);
    }

    Quaternion operator+() const { return *this; }

    Quaternion operator-() const { return {-w_, -x_, -y_, -z_}; }

    Quaternion operator+=(T w) {
        w_ += w;
        return *this;
    }

    Quaternion operator-=(T w) {
        w_ -= w;
        return *this;
    }

    Quaternion operator*=(T k) {
        w_ = k * w_;
        x_ = k * x_;
        y_ = k * y_;
        z_ = k * z_;
        return *this;
    }

    Quaternion operator/=(T k) {
        w_ /= k;
        x_ /= k;
        y_ /= k;
        z_ /= k;
        return *this;
    }

    template <typename T1>
    Quaternion operator+=(Quaternion<T1> const& other) {
        w_ += other.w();
        x_ += other.x();
        y_ += other.y();
        z_ += other.z();
        return *this;
    }

    template <typename T1>
    Quaternion operator-=(Quaternion<T1> const& other) {
        w_ -= other.w();
        x_ -= other.x();
        y_ -= other.y();
        z_ -= other.z();
        return *this;
    }

    template <typename T1>
    Quaternion operator*=(Quaternion<T1> const& other) {
        T w = w_ * other.w() - x_ * other.x() - y_ * other.y() - z_ * other.z();
        T x = w_ * other.x() + x_ * other.w() + y_ * other.z() - z_ * other.y();
        T y = w_ * other.y() - x_ * other.z() + y_ * other.w() + z_ * other.x();
        T z = w_ * other.z() + x_ * other.y() - y_ * other.x() + z_ * other.w();

        w_ = w;
        x_ = x;
        y_ = y;
        z_ = z;

        return *this;
    }

    template <typename T1>
    Quaternion operator/=(const Quaternion<T1>& other) {
        T n2 = other.norm_squared();

        T w = w_ * other.w() + x_ * other.x() + y_ * other.y() + z_ * other.z();
        T x = -w_ * other.x() + x_ * other.w() - y_ * other.z() + z_ * other.y();
        T y = -w_ * other.y() + x_ * other.z() + y_ * other.w() - z_ * other.x();
        T z = -w_ * other.z() - x_ * other.y() + y_ * other.x() + z_ * other.w();

        w_ = w / n2;
        x_ = x / n2;
        y_ = y / n2;
        z_ = z / n2;

        return *this;
    }

 private:
    T w_, x_, y_, z_;  // the full state for a Quaternion
};

/**
 * Returns three Euler angles {yaw, pitch, roll} in radians.
 * x is required to be a unit quaternion.
 *
 * WARNING: conversion to/from Euler angles is not readother.
 */
template <typename T>
inline EulerAngles<T> to_euler(Quaternion<T> const& q, T eps = 1e-7) {
    // Normalize quaternion to be safe
    auto norm = std::sqrt(q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
    T w = q.w() / norm;
    T x = q.x() / norm;
    T y = q.y() / norm;
    T z = q.z() / norm;

    T roll, pitch, yaw;

    // roll (x-axis rotation)
    T sinr_cosp = 2 * (w * x + y * z);
    T cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    T sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    T siny_cosp = 2 * (w * z + x * y);
    T cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    return EulerAngles<T>{roll, pitch, yaw};
}

/**
 * Returns a unit quaternion corresponding to the three Euler angles
 * {yaw, pitch, roll} expressed in radians.
 * The conventions used are with the 3,2,1 convention ??? TODO: verify
 */
template <typename T>
inline Maybe<Quaternion<T> > from_euler(EulerAngles<T> const& x) {
    T cy = std::cos(x.yaw() * 0.5);
    T sy = std::sin(x.yaw() * 0.5);
    T cp = std::cos(x.pitch() * 0.5);
    T sp = std::sin(x.pitch() * 0.5);
    T cr = std::cos(x.roll() * 0.5);
    T sr = std::sin(x.roll() * 0.5);

    Quaternion<T> q{
        cr * cp * cy + sr * sp * sy,  // w
        sr * cp * cy - cr * sp * sy,  // x
        cr * sp * cy + sr * cp * sy,  // y
        cr * cp * sy - sr * sp * cy   // z
    };
    return normalize(q);
}

/** +
 * Returns the conjugate of x, as a new Quaternion (x is unchanged).
 */
template <typename T>
inline Quaternion<T> conj(const Quaternion<T>& x) {
    return {x.w(), -x.x(), -x.y(), -x.z()};
}

/** +
 * Norms on a Quaternion.
 */
template <typename T>
inline T norm_squared(const Quaternion<T>& x) {
    return x.norm_squared();
}

template <typename T>
inline T complex_norm_squared(const Quaternion<T>& x) {
    return x.complex_norm_squared();
}

// Hamming
template <typename T>
inline T norm_l0(const Quaternion<T>& x) {
    return (x.w() != 0) + (x.x() != 0) + (x.y() != 0) + (x.z() != 0);
}

// l1 norm = taxicab = manhattan
template <typename T>
inline T norm_l1(const Quaternion<T>& x) {
    return std::abs(x.w()) + std::abs(x.x()) + std::abs(x.y()) + std::abs(x.z());
}

template <typename T, typename T1>
inline T norm_lk(const Quaternion<T>& x, T1 k) {
    return std::pow(std::pow(std::abs(x.w()), k) + std::pow(std::abs(x.x()), k) +
                        std::pow(std::abs(x.y()), k) + std::pow(std::abs(x.z()), k),
                    1.0 / k);
}

// norm sup = max norm = norm inf
template <typename T>
inline T norm_sup(const Quaternion<T>& x) {
    return std::max(std::max(std::abs(x.w()), std::abs(x.x())),
                    std::max(std::abs(x.y()), std::abs(x.z())));
}

// abs = l2 norm = euclidean norm
template <typename T>
inline T abs(const Quaternion<T>& x) {
    return norm_lk(x, 2);
}

/**
 * Quaternion tests.
 */
template <typename T, typename T1 = T>
inline bool is_zero(const Quaternion<T>& x, T1 eps = 0) {
    return x.is_zero(eps);
}

template <typename T>
inline bool is_nan(const Quaternion<T>& x) {
    return x.is_nan();
}

template <typename T>
inline bool is_inf(const Quaternion<T>& x) {
    return x.is_inf();
}

template <typename T>
inline bool is_finite(const Quaternion<T>& x) {
    return x.is_finite();
}

template <typename T, typename T1 = T>
inline bool is_unit(const Quaternion<T>& x, T1 eps = 0) {
    return x.is_unit(eps);
}

template <typename T, typename T1 = T>
inline bool is_real(const Quaternion<T>& x, T1 eps = 0) {
    return x.is_real(eps);
}

template <typename T, typename T1 = T>
inline bool is_complex(const Quaternion<T>& x, T1 eps = 0) {
    return x.is_complex(eps);
}

template <typename T, typename T2>
inline bool operator==(const Quaternion<T>& x, T2 y) {
    return x.is_real() && x.w() == y;
}

template <typename T, typename T2>
inline bool operator==(T2 y, const Quaternion<T>& x) {
    return x == y;
}

template <typename T, typename T2>
inline bool operator!=(const Quaternion<T>& x, T2 y) {
    return !(x == y);
}

template <typename T, typename T2>
inline bool operator!=(T2 y, const Quaternion<T>& x) {
    return !(x == y);
}

/**
 * Quaternion <-> Quaternion
 */
template <typename T1, typename T2>
inline bool operator==(const Quaternion<T1>& x, const Quaternion<T2>& other) {
    return x.w() == other.w() && x.x() == other.x() && x.y() == other.y() && x.z() == other.z();
}

template <typename T>
inline bool operator!=(const Quaternion<T>& x, const Quaternion<T>& y) {
    return !(x == y);
}

template <typename T, typename T2>
inline bool nearly_equal(const Quaternion<T>& q1, const Quaternion<T>& q2, T2 tol) {
    // Quaternions represent same rotation if q1 == q2 or q1 == -q2
    return (q1 - q2).norm() < tol || (q1 + q2).norm() < tol;
}

template <typename T, typename T1>
inline Quaternion<T> operator+(const Quaternion<T>& x, T1 y) {
    return Quaternion<T>(x) += y;
}

template <typename T, typename T1>
inline Quaternion<T> operator+(T1 y, const Quaternion<T>& x) {
    return x + y;
}

template <typename T>
inline Quaternion<T> operator+(const Quaternion<T>& x, const Quaternion<T>& y) {
    return Quaternion<T>(x) += y;
}

template <typename T, typename T1>
inline Quaternion<T> operator-(const Quaternion<T>& x, T1 y) {
    return Quaternion<T>(x) -= y;
}

template <typename T, typename T1>
inline Quaternion<T> operator-(T1 y, const Quaternion<T>& x) {
    return Quaternion<T>(x) += -y;
}

template <typename T>
inline Quaternion<T> operator-(const Quaternion<T>& x, const Quaternion<T>& y) {
    return Quaternion<T>(x) -= y;
}

/**
 * SSE operations: tried 2 implementations (SO and vectorclass): not faster.
 * Boost: as fast as boost implementation.
 */
template <typename T, typename T1>
inline Quaternion<T> operator*(const Quaternion<T>& x, T1 y) {
    return Quaternion<T>(x) *= y;
}

template <typename T, typename T1>
inline Quaternion<T> operator*(T1 y, const Quaternion<T>& x) {
    return x * y;
}

template <typename T>
inline Quaternion<T> operator*(const Quaternion<T>& x, const Quaternion<T>& y) {
    return Quaternion<T>(x) *= y;
}

template <typename T>
inline Quaternion<T> inverse(const Quaternion<T>& x) {
    return conj(x) / norm_squared(x);
}

template <typename T, typename T1>
inline Quaternion<T> operator/(const Quaternion<T>& x, T1 y) {
    return Quaternion<T>(x) /= y;
}

template <typename T, typename T1>
inline Quaternion<T> operator/(T1 y, const Quaternion<T>& x) {
    return y * inverse(x);
}

template <typename T>
inline Quaternion<T> operator/(const Quaternion<T>& x, const Quaternion<T>& y) {
    return x * inverse(y);
}

template <typename T>
inline T dot(const Quaternion<T>& x, const Quaternion<T>& other) {
    return x.w() * other.w() + x.x() * other.x() + x.y() * other.y() + x.z() * other.z();
}

/**
 * 9 operations
 */
template <typename T>
inline Quaternion<T> cross(const Quaternion<T>& x, const Quaternion<T>& other) {
    return {0, x.y() * other.z() - x.z() * other.y(), x.z() * other.x() - x.x() * other.z(),
            x.x() * other.y() - x.y() * other.x()};
}

template <typename T>
inline Quaternion<T> commutator(const Quaternion<T>& x, const Quaternion<T>& y) {
    return x * y - y * x;
}

template <typename T>
inline Maybe<Quaternion<T> > normalize(const Quaternion<T>& x) {
    if (x.norm() < 1e-12) {
        return Nothing<Quaternion<T> >();
    }

    return Just(x / x.norm());
}

template <typename T>
inline void rotate(Quaternion<T> const& q, Vector3<T>& v) {
    Quaternion<T> const p{v};
    auto const result{q * p * conj(q)};

    v.set(result.x(), result.y(), result.z());
}

template <typename T>
inline Vector3<T> rotated(Quaternion<T> const& q, Vector3<T> const& v) {
    Quaternion<T> const p{v};
    auto const result{q * p * conj(q)};

    return {result.x(), result.y(), result.z()};
}
}  // namespace geometry
}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_GEOMETRY_QUATERNION_H_
