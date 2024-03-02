// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_GEOMETRY_VECTOR_H_
#define LINE_FOLLOWER_BLOCKS_GEOMETRY_VECTOR_H_

namespace line_follower {
namespace geometry {
/// @brief A 3-element vector class
/// @tparam T Any numeric type
template <class T>
class Vector3 {
 public:
    typedef T value_type;

    static_assert(std::is_same<T, float>() || std::is_same<T, double>(),
                  "Invalid scalar type for Vector3");

    Vector3();
    Vector3(const T x, const T y, const T z);
    Vector3(const Vector3<T>& v);

    T x() const { return _v[0]; }

    T y() const { return _v[1]; }

    T z() const { return _v[2]; }

    // utility operations
    Vector3<T>& zero();
    Vector3<T>& set(const T x, const T y, const T z);
    Vector3<T>& normalize();

    // math operations
    const T norm() const;
    const T sum() const;
    const T dot(const Vector3<T>&) const;
    const Vector3<T> cross(const Vector3<T>&) const;
    const Vector3<T> abs() const;

    Vector3<T>& operator=(const Vector3<T>& v);

    const T operator[](const int i) const;
    T& operator[](const int i);

    const Vector3<T> operator-();

    Vector3<T>& operator+=(const T s);
    Vector3<T>& operator-=(const T s);
    Vector3<T>& operator*=(const T s);
    Vector3<T>& operator/=(const T s);

    Vector3<T>& operator+=(const Vector3<T>& v);
    Vector3<T>& operator-=(const Vector3<T>& v);
    Vector3<T>& operator*=(const Vector3<T>& v);
    Vector3<T>& operator/=(const Vector3<T>& v);

    const Vector3<T> operator<(const T s);
    const Vector3<T> operator>(const T s);

    const Vector3<T> operator<(const Vector3<T>& v);
    const Vector3<T> operator>(const Vector3<T>& v);

    bool operator==(const Vector3<T>& v);
    bool operator!=(const Vector3<T>& v);

    T* ptr() { return _v; }

 private:
    T _v[3];
};

template <class T>
Vector3<T>::Vector3() {
    _v[0] = 0.0;
    _v[1] = 0.0;
    _v[2] = 0.0;
}

template <class T>
Vector3<T>::Vector3(const Vector3<T>& v) {
    _v[0] = v[0];
    _v[1] = v[1];
    _v[2] = v[2];
}

template <class T>
Vector3<T>::Vector3(const T x, const T y, const T z) {
    _v[0] = x;
    _v[1] = y;
    _v[2] = z;
}

template <class T>
Vector3<T>& Vector3<T>::zero() {
    _v[0] = 0.0;
    _v[1] = 0.0;
    _v[2] = 0.0;
    return *this;
}

template <class T>
Vector3<T>& Vector3<T>::set(const T x, const T y, const T z) {
    _v[0] = x;
    _v[1] = y;
    _v[2] = z;
    return *this;
}

template <class T>
inline const T Vector3<T>::operator[](const int i) const {
    return _v[i];
}

template <class T>
T& Vector3<T>::operator[](const int i) {
    return _v[i];
}

template <class T>
inline const Vector3<T> Vector3<T>::abs() const {
    return Vector3<T>(std::abs(_v[0]), std::abs(_v[1]), std::abs(_v[2]));
}

template <class T>
inline const T Vector3<T>::sum() const {
    return _v[0] + _v[1] + _v[2];
}

template <class T>
inline const T Vector3<T>::dot(const Vector3<T>& v) const {
    return _v[0] * v[0] + _v[1] * v[1] + _v[2] * v[2];
}

template <class T>
inline const Vector3<T> Vector3<T>::cross(const Vector3<T>& v) const {
    return Vector3<T>((_v[1] * v[2]) - (_v[2] * v[1]), (_v[2] * v[0]) - (_v[0] * v[2]),
                      (_v[0] * v[1]) - (_v[1] * v[0]));
}

template <class T>
inline const T Vector3<T>::norm() const {
    return (T)sqrt(dot(*this));  // cast to type
}

template <class T>
Vector3<T>& Vector3<T>::normalize() {
    T n = norm();

    if (n) {
        _v[0] /= n;
        _v[1] /= n;
        _v[2] /= n;
    }

    return *this;
}

template <class T>
Vector3<T> normalized(Vector3<T> const& v) {
    Vector3<T> normalized_v{v};

    static_cast<void>(normalized_v.normalize());
    return normalized_v;
}

template <class T>
Vector3<T> lerp(Vector3<T> a, Vector3<T> b, double f) {
    return a * (1.0 - f) + (b * f);
}

template <class T>
Vector3<T>& Vector3<T>::operator=(const Vector3<T>& v) {
    _v[0] = v[0];
    _v[1] = v[1];
    _v[2] = v[2];
    return *this;
}

template <class T>
Vector3<T>& Vector3<T>::operator+=(const Vector3<T>& v) {
    _v[0] += v[0];
    _v[1] += v[1];
    _v[2] += v[2];
    return *this;
}

template <class T>
Vector3<T>& Vector3<T>::operator+=(T v) {
    _v[0] += v;
    _v[1] += v;
    _v[2] += v;
    return *this;
}

template <class T>
Vector3<T>& Vector3<T>::operator-=(const Vector3<T>& v) {
    _v[0] -= v[0];
    _v[1] -= v[1];
    _v[2] -= v[2];
    return *this;
}

template <class T>
Vector3<T>& Vector3<T>::operator-=(T v) {
    _v[0] -= v;
    _v[1] -= v;
    _v[2] -= v;
    return *this;
}

template <class T>
Vector3<T>& Vector3<T>::operator*=(T v) {
    _v[0] *= v;
    _v[1] *= v;
    _v[2] *= v;
    return *this;
}

template <class T>
Vector3<T>& Vector3<T>::operator*=(const Vector3<T>& v) {
    _v[0] *= v[0];
    _v[1] *= v[1];
    _v[2] *= v[2];
    return *this;
}

template <class T>
Vector3<T>& Vector3<T>::operator/=(T v) {
    _v[0] /= v;
    _v[1] /= v;
    _v[2] /= v;
    return *this;
}

template <class T>
Vector3<T>& Vector3<T>::operator/=(const Vector3<T>& v) {
    _v[0] /= v[0];
    _v[1] /= v[1];
    _v[2] /= v[2];
    return *this;
}

template <class T>
inline const Vector3<T> Vector3<T>::operator<(const T s) {
    return Vector3<T>(_v[0] < s, _v[1] < s, _v[2] < s);
}

template <class T>
inline const Vector3<T> Vector3<T>::operator>(const T s) {
    return Vector3<T>(_v[0] > s, _v[1] > s, _v[2] > s);
}

template <class T>
inline const Vector3<T> Vector3<T>::operator<(const Vector3<T>& v) {
    return Vector3<T>(_v[0] < v[0], _v[1] < v[1], _v[2] < v[2]);
}

template <class T>
inline const Vector3<T> Vector3<T>::operator>(const Vector3<T>& v) {
    return Vector3<T>(_v[0] > v[0], _v[1] > v[1], _v[2] > v[2]);
}

template <class T>
inline const Vector3<T> Vector3<T>::operator-() {
    return Vector3<T>(-_v[0], -_v[1], -_v[2]);
}

template <class T>
inline bool Vector3<T>::operator==(const Vector3<T>& v) {
    return _v[0] == v[0] && _v[1] == v[1] && _v[2] == v[2];
}

template <class T>
inline bool Vector3<T>::operator!=(const Vector3<T>& v) {
    return _v[0] != v[0] || _v[1] != v[1] || _v[2] != v[2];
}

template <class T>
inline const Vector3<T> operator&&(const Vector3<T>& v1, const Vector3<T>& v2) {
    return Vector3<T>(v1[0] && v2[0], v1[1] && v2[1], v1[2] && v2[2]);
}

template <class T>
inline const Vector3<T> operator||(const Vector3<T>& v1, const Vector3<T>& v2) {
    return Vector3<T>(v1[0] || v2[0], v1[1] || v2[1], v1[2] || v2[2]);
}

template <class T>
inline const Vector3<T> operator+(const Vector3<T>& v, const T& s) {
    return Vector3<T>(v) += s;
}

template <class T>
inline const Vector3<T> operator-(const Vector3<T>& v, const T& s) {
    return Vector3<T>(v) -= s;
}

template <class T>
inline const Vector3<T> operator*(const Vector3<T>& v, const T& s) {
    return Vector3<T>(v) *= s;
}

template <class T>
inline const Vector3<T> operator/(const Vector3<T>& v, const T& s) {
    return Vector3<T>(v) /= s;
}

template <class T>
inline const Vector3<T> operator+(const Vector3<T>& v1, const Vector3<T>& v2) {
    return Vector3<T>(v1) += v2;
}

template <class T>
inline const Vector3<T> operator-(const Vector3<T>& v1, const Vector3<T>& v2) {
    return Vector3<T>(v1) -= v2;
}

template <class T>
inline const T operator*(const Vector3<T>& v1, const Vector3<T>& v2) {
    return v1.dot(v2);
}

template <class T>
inline const Vector3<T> operator^(const Vector3<T>& v1, const Vector3<T>& v2) {
    return v1.cross(v2);
}
}  // namespace geometry
}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_GEOMETRY_VECTOR_H_
