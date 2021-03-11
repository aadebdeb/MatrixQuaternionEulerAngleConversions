#ifndef __QUATERNION_H__
#define __QUATERNION_H__

#include <cmath>

#include "./Vector3.h"

class Quaternion {
public:
  float x;
  float y;
  float z;
  float w;
  Quaternion(float x, float y, float z, float w): x(x), y(y),z(z), w(w) {}
  static Quaternion rotationX(float angle);
  static Quaternion rotationY(float angle);
  static Quaternion rotationZ(float angle);
  Quaternion operator*(const Quaternion q) const;
  Vector3 rotate(const Vector3 v) const;
};

Quaternion Quaternion::rotationX(float angle) {
  return Quaternion(std::sin(0.5f * angle), 0, 0, std::cos(0.5f * angle));
}

Quaternion Quaternion::rotationY(float angle) {
  return Quaternion(0, std::sin(0.5f * angle), 0, std::cos(0.5f * angle));
}

Quaternion Quaternion::rotationZ(float angle) {
  return Quaternion(0, 0, std::sin(0.5f * angle), std::cos(0.5f * angle));
}

Quaternion conjugate(const Quaternion q) {
  return Quaternion(-q.x, -q.y, -q.z, q.w);
}

Quaternion Quaternion::operator*(const Quaternion q) const {
  return Quaternion(
    w * q.x - z * q.y + y * q.z + x * q.w,
    z * q.x + w * q.y - x * q.z + y * q.w,
    -y * q.x + x * q.y + w * q.z + z * q.w,
    -x * q.x - y * q.y - z * q.z + w * q.w
  );
}

Vector3 Quaternion::rotate(const Vector3 v) const {
  auto vq = Quaternion(v.x, v.y, v.z, 0);
  auto cq = conjugate(*this);
  auto mq = *this * vq * cq;
  return Vector3(mq.x, mq.y, mq.z);
}

#endif // __QUATERNION_H__