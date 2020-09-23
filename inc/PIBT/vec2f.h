/*
 * This class was coded referring to ofVec2f class in openFrameworks.
 * https://github.com/openframeworks/openFrameworks/blob/master/libs/openFrameworks/math/ofVec2f.h
 */

#pragma once
#include <iostream>
#include <math.h>

class Vec2f {
public:
  float x;
  float y;

  Vec2f();
  Vec2f(float x, float y);

  void set(float x, float y);
  void set(const Vec2f vec);

  bool operator==(const Vec2f& vec) const;
  bool operator!=(const Vec2f& vec) const;
  Vec2f operator+(const Vec2f& vec) const;
  Vec2f& operator+=(const Vec2f& vec);
  Vec2f operator-(const Vec2f& vec) const;
  Vec2f& operator-=(const Vec2f& vec);
  Vec2f operator*(const Vec2f& vec) const;
  Vec2f operator*(const float f) const;
  Vec2f& operator*=(const Vec2f& vec);
  Vec2f& operator*=(const float f);
  Vec2f operator/(const Vec2f& vec) const;
  Vec2f operator/(const float f) const;
  Vec2f& operator/=(const Vec2f& vec);
  Vec2f& operator/=(const float f);

  float distance(const Vec2f& vec) const;
  float length() const;

  friend std::ostream& operator<<(std::ostream& os, const Vec2f& vec);
};


inline Vec2f::Vec2f(): x(0), y(0) {}
inline Vec2f::Vec2f(float _x, float _y): x(_x), y(_y) {}

inline void Vec2f::set(float _x, float _y) {
  x = _x;
  y = _y;
}
inline void Vec2f::set(Vec2f vec) {
  x = vec.x;
  y = vec.y;
}

inline bool Vec2f::operator==(const Vec2f& vec) const {
  return (x == vec.x) && (y == vec.y);
}
inline bool Vec2f::operator!=(const Vec2f& vec) const {
  return (x != vec.x) || (y != vec.y);
}

inline Vec2f Vec2f::operator+(const Vec2f& vec) const {
  return Vec2f(x + vec.x, y + vec.y);
}
inline Vec2f& Vec2f::operator+=(const Vec2f& vec) {
  x += vec.x;
  y += vec.y;
  return *this;
}

inline Vec2f Vec2f::operator-(const Vec2f& vec) const {
  return Vec2f(x - vec.x, y - vec.y);
}
inline Vec2f& Vec2f::operator-=(const Vec2f& vec) {
  x -= vec.x;
  y -= vec.y;
  return *this;
}

inline Vec2f Vec2f::operator*(const Vec2f& vec) const {
  return Vec2f(x * vec.x, y * vec.y);
}
inline Vec2f Vec2f::operator*(const float f) const {
  return Vec2f(x * f, y * f);
}
inline Vec2f& Vec2f::operator*=(const Vec2f& vec) {
  x *= vec.x;
  y *= vec.y;
  return *this;
}
inline Vec2f& Vec2f::operator*=(const float f) {
  x *= f;
  y *= f;
  return *this;
}

inline Vec2f Vec2f::operator/(const Vec2f& vec) const {
  return Vec2f(x / vec.x, y / vec.y);
}
inline Vec2f Vec2f::operator/(const float f) const {
  return Vec2f(x / f, y / f);
}
inline Vec2f& Vec2f::operator/=(const Vec2f& vec) {
  x /= vec.x;
  y /= vec.y;
  return *this;
}
inline Vec2f& Vec2f::operator/=(const float f) {
  x /= f;
  y /= f;
  return *this;
}

inline float Vec2f::distance(const Vec2f& vec) const {
  float vx = x - vec.x;
  float vy = y - vec.y;
  return (float)sqrt(vx * vx + vy * vy);
}

inline float Vec2f::length() const {
  return (float)sqrt(x * x + y * y);
}

inline std::ostream& operator<<(std::ostream& os, const Vec2f& vec) {
  os << "(" << vec.x << ", " << vec.y << ")";
  return os;
}
