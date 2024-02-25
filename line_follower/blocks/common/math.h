// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_COMMON_MATH_H_
#define LINE_FOLLOWER_BLOCKS_COMMON_MATH_H_

#include <cmath>
#include <type_traits>

namespace line_follower
{

/// Signum function
template <typename T>
inline int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

/// Convert radians to degrees
template <typename T>
inline T radToDeg(T const& radians)
{
  return radians * 180.0 / M_PI;
}

/// Convert degrees to radians
template <typename T>
inline T degToRad(T const& degrees)
{
  return degrees * M_PI / 180.0;
}

/// Utility function to work with numbers approximately equal to zero.
/// If eps == 0, does a "hard" comparison to 0.
/// Otherwise, uses a ball of radius eps around 0. If the scalar is inside
/// that ball, it is equivalent to 0.
template<typename T1, typename T2>
inline bool is_scalar_zero(T1 x, T2 eps = 0) {
  typedef typename std::common_type<T1,T2>::type T;
  T xx = static_cast<T>(x);
  T ee = static_cast<T>(eps);
  return std::abs(xx) <= ee;
}

/// Compares 2 floating point numbers "relatively": if the numbers are
/// very large, differences are still "small" if they are "small"
/// relative to the magnitudes of the quantities.
/// TODO: need absolute difference comparison (not relative) too
template<typename T1, typename T2, typename T3>
inline bool is_nearly_equal(T1 x, T2 y, T3 eps = 0) {
  typedef typename std::common_type<T1,T2,T3>::type T;
  T xx = static_cast<T>(x);
  T yy = static_cast<T>(y);
  T ee = static_cast<T>(eps);
  if (xx == 0)
    return is_scalar_zero(yy, ee);
  else if (yy == 0)
    return is_scalar_zero(xx, ee);
  else
    return is_scalar_zero((xx - yy) / std::min(xx, yy), ee);
}

}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_COMMON_MATH_H_