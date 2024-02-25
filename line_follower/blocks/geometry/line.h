// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_GEOMETRY_LINE_H_
#define LINE_FOLLOWER_BLOCKS_GEOMETRY_LINE_H_

#include <vector>

#include "line_follower/blocks/geometry/vector.h"

namespace line_follower
{

namespace geometry
{

/// @brief A 3-dimensional line class
/// @tparam T Any numeric type
template<class T>
class Line
{
public:
  typedef T value_type;

  static_assert(std::is_same<T, float>()
                || std::is_same<T, double>(),
                "Invalid scalar type for Line");

  Line()
  {
    from_ = Vector3<T>{0.0, 0.0, 0.0};
    to_ = Vector3<T>{0.0, 0.0, 0.0};
  }

  /// @brief Constructor for the Line class
  /// @param from From 3-dimensional point
  /// @param to To 3-dimensional point
  Line(Vector3<T> const& from, Vector3<T> const& to)
  {
    from_ = from;
    to_ = to;
  }

  /// @brief Constructor for the Line class
  /// @param v A vector describing the line, from zero origin
  Line(Vector3<T> const& v)
  {
    from_ = Vector3<T>{0.0, 0.0, 0.0};
    to_ = v;
  }

  Line(Line<T> const& line)
  {
    from_ = line.from();
    to_ = line.to();
  }

  Vector3<T> const& from() const { return from_; }
  Vector3<T> const& to() const { return to_; }
  Vector3<T> const center() const { return (to_ + from_) / 2.0; }

  void set(Vector3<T> const& from, Vector3<T> const& to)
  {
    from_ = from;
    to_ = to;
  }

  bool intersectsWith(Line<T> const& line);
  bool intersectsWithAny(std::vector<Line<T>> const& lines);
  bool intersectsWithAll(std::vector<Line<T>> const& lines);

  const Line<T> operator-()
  {
    return Line<T>(to_, from_);
  }

  bool operator == (const Line<T>& line)
  {
    return    from_ == line.from()
          && to_ == line.to();
  }

  bool operator != (const Line<T>& line)
  {
    return !(*this == line);
  }

private:
  Vector3<T> from_;
  Vector3<T> to_;
};

template <class T>
bool Line<T>::intersectsWith(Line<T> const& line)
{
    auto a{*this};
    auto b{line};
    double p0_x = a.from().x();
    double p1_x = a.to().x();
    double p2_x = b.from().x();
    double p3_x = b.to().x();

    double p0_y = a.from().y();
    double p1_y = a.to().y();
    double p2_y = b.from().y();
    double p3_y = b.to().y();

    double s1_x, s1_y, s2_x, s2_y;
    s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

    double s, t;
    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
    {
        return true;  // Intersection detected
    }

    return false;  // No intersection
}

template <class T>
bool Line<T>::intersectsWithAny(std::vector<Line<T>> const& lines)
{
  for (auto const& line : lines)
  {
    if (intersectsWith(line))
    {
      return true;
    }
  }
  return false;
}

template <class T>
bool Line<T>::intersectsWithAll(std::vector<Line<T>> const& lines)
{
  for (auto const& line : lines)
  {
    if (!intersectsWith(line))
    {
      return false;
    }
  }
  return true;
}

}  // namespace geometry
}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_GEOMETRY_LINE_H_