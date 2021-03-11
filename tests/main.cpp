#include <iostream>
#include <cmath>

#include <gtest/gtest.h>

#include "../src/EulerAngle.h"
#include "../src/Quaternion.h"
#include "../src/RotationMatrix.h"
#include "../src/Vector3.h"
#include "../src/conversion.h"

const float PI = 3.14159265359f;
const float HALF_PI = 0.5f * PI;
const float TWO_PI = 2 * PI;

void printVector3(const Vector3 v) {
  std::cout << "Vector3(" << v.x << "," << v.y << "," << v.z << ")" << std::endl;
}

void printEulerAngle(const EulerAngle e) {
  std::cout << "EulerAngle(" << e.x << "," << e.y << "," << e.z << ")" << std::endl;
}

void printQuaternion(const Quaternion q) {
  std::cout << "Quaternion(" << q.x << "," << q.y << "," << q.z << "," << q.w << ")" << std::endl;
}

void printRotationMatrix(RotationMatrix m) {
  std::cout << "RotationMatrix(\n  "
    << m.at(0, 0) << "," << m.at(0, 1) << "," << m.at(0, 2) << ",\n  "
    << m.at(1, 0) << "," << m.at(1, 1) << "," << m.at(2, 2) << ",\n  "
    << m.at(2, 0) << "," << m.at(2, 1) << "," << m.at(2, 2) << "\n)"  << std::endl;
}

bool equals(const Vector3 v1, const Vector3 v2, float error = 0.02) {
  return std::abs(v1.x - v2.x) < error && std::abs(v1.y - v2.y) < error && std::abs(v1.z - v2.z) < error;
}

RotationMatrix calculateRotationMatrix(const EulerAngle e) {
  auto x = RotationMatrix::rotationX(e.x);
  auto y = RotationMatrix::rotationY(e.y);
  auto z = RotationMatrix::rotationZ(e.z);
  switch (e.order) {
  case EulerOrder::XYZ:
    return x * y * z;
  case EulerOrder::XZY:
    return x * z * y;
  case EulerOrder::YXZ:
    return y * x * z;
  case EulerOrder::YZX:
    return y * z * x;
  case EulerOrder::ZXY:
    return z * x * y;
  case EulerOrder::ZYX:
    return z * y * x;
  }
  throw "order of euler angle does not matched.";
}

Quaternion calculateQuaternion(const EulerAngle e) {
  auto x = Quaternion::rotationX(e.x);
  auto y = Quaternion::rotationY(e.y);
  auto z = Quaternion::rotationZ(e.z);
  switch (e.order) {
  case EulerOrder::XYZ:
    return x * y * z;
  case EulerOrder::XZY:
    return x * z * y;
  case EulerOrder::YXZ:
    return y * x * z;
  case EulerOrder::YZX:
    return y * z * x;
  case EulerOrder::ZXY:
    return z * x * y;
  case EulerOrder::ZYX:
    return z * y * x;
  }
  throw "order of euler angle does not matched.";
}

TEST(QuaternionToEulerAngle, XYZ) {
  auto q00 = calculateQuaternion(EulerAngle(0, 0, 0, EulerOrder::XYZ));
  auto e00 = toEulerAngle(q00, EulerOrder::XYZ);
  auto m00 = calculateRotationMatrix(e00);
  auto q01 = calculateQuaternion(EulerAngle(PI * 0.333, PI * 0.777, PI * 1.222, EulerOrder::XYZ));
  auto e01 = toEulerAngle(q01, EulerOrder::XYZ);
  auto m01 = calculateRotationMatrix(e01);
  auto q02 = calculateQuaternion(EulerAngle(PI * 0.777, PI * 1.222, PI * 0.333, EulerOrder::XYZ));
  auto e02 = toEulerAngle(q02, EulerOrder::XYZ);
  auto m02 = calculateRotationMatrix(e02);
  auto q03 = calculateQuaternion(EulerAngle(PI * 1.222, PI * 0.333, PI * 0.777, EulerOrder::XYZ));
  auto e03 = toEulerAngle(q03, EulerOrder::XYZ);
  auto m03 = calculateRotationMatrix(e03);
  auto q04 = calculateQuaternion(EulerAngle(PI * 0.333, 0.5 * PI, PI * 1.777, EulerOrder::XYZ));
  auto e04 = toEulerAngle(q04, EulerOrder::XYZ);
  auto m04 = calculateRotationMatrix(e04);
  auto q05 = calculateQuaternion(EulerAngle(-PI * 1.333, -0.5 * PI, -PI * 0.777, EulerOrder::XYZ));
  auto e05 = toEulerAngle(q05, EulerOrder::XYZ);
  auto m05 = calculateRotationMatrix(e05);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(q00.rotate(v000), m00 * v000));
  EXPECT_TRUE(equals(q01.rotate(v000), m01 * v000));
  EXPECT_TRUE(equals(q02.rotate(v000), m02 * v000));
  EXPECT_TRUE(equals(q03.rotate(v000), m03 * v000));
  EXPECT_TRUE(equals(q04.rotate(v000), m04 * v000));
  EXPECT_TRUE(equals(q05.rotate(v000), m05 * v000));
  EXPECT_TRUE(equals(q00.rotate(v100), m00 * v100));
  EXPECT_TRUE(equals(q01.rotate(v100), m01 * v100));
  EXPECT_TRUE(equals(q02.rotate(v100), m02 * v100));
  EXPECT_TRUE(equals(q03.rotate(v100), m03 * v100));
  EXPECT_TRUE(equals(q04.rotate(v100), m04 * v100));
  EXPECT_TRUE(equals(q05.rotate(v100), m05 * v100));
  EXPECT_TRUE(equals(q00.rotate(v010), m00 * v010));
  EXPECT_TRUE(equals(q01.rotate(v010), m01 * v010));
  EXPECT_TRUE(equals(q02.rotate(v010), m02 * v010));
  EXPECT_TRUE(equals(q03.rotate(v010), m03 * v010));
  EXPECT_TRUE(equals(q04.rotate(v010), m04 * v010));
  EXPECT_TRUE(equals(q05.rotate(v010), m05 * v010));
  EXPECT_TRUE(equals(q00.rotate(v001), m00 * v001));
  EXPECT_TRUE(equals(q01.rotate(v001), m01 * v001));
  EXPECT_TRUE(equals(q02.rotate(v001), m02 * v001));
  EXPECT_TRUE(equals(q03.rotate(v001), m03 * v001));
  EXPECT_TRUE(equals(q04.rotate(v001), m04 * v001));
  EXPECT_TRUE(equals(q05.rotate(v001), m05 * v001));
  EXPECT_TRUE(equals(q00.rotate(va), m00 * va));
  EXPECT_TRUE(equals(q01.rotate(va), m01 * va));
  EXPECT_TRUE(equals(q02.rotate(va), m02 * va));
  EXPECT_TRUE(equals(q03.rotate(va), m03 * va));
  EXPECT_TRUE(equals(q04.rotate(va), m04 * va));
  EXPECT_TRUE(equals(q05.rotate(va), m05 * va));
  EXPECT_TRUE(equals(q00.rotate(vb), m00 * vb));
  EXPECT_TRUE(equals(q01.rotate(vb), m01 * vb));
  EXPECT_TRUE(equals(q02.rotate(vb), m02 * vb));
  EXPECT_TRUE(equals(q03.rotate(vb), m03 * vb));
  EXPECT_TRUE(equals(q04.rotate(vb), m04 * vb));
  EXPECT_TRUE(equals(q05.rotate(vb), m05 * vb));
  EXPECT_TRUE(equals(q00.rotate(vc), m00 * vc));
  EXPECT_TRUE(equals(q01.rotate(vc), m01 * vc));
  EXPECT_TRUE(equals(q02.rotate(vc), m02 * vc));
  EXPECT_TRUE(equals(q03.rotate(vc), m03 * vc));
  EXPECT_TRUE(equals(q04.rotate(vc), m04 * vc));
  EXPECT_TRUE(equals(q05.rotate(vc), m05 * vc));
}

TEST(QuaternionToEulerAngle, XZY) {
  auto q00 = calculateQuaternion(EulerAngle(0, 0, 0, EulerOrder::XZY));
  auto e00 = toEulerAngle(q00, EulerOrder::XZY);
  auto m00 = calculateRotationMatrix(e00);
  auto q01 = calculateQuaternion(EulerAngle(PI * 0.333, PI * 0.777, PI * 1.222, EulerOrder::XZY));
  auto e01 = toEulerAngle(q01, EulerOrder::XZY);
  auto m01 = calculateRotationMatrix(e01);
  auto q02 = calculateQuaternion(EulerAngle(PI * 0.777, PI * 1.222, PI * 0.333, EulerOrder::XZY));
  auto e02 = toEulerAngle(q02, EulerOrder::XZY);
  auto m02 = calculateRotationMatrix(e02);
  auto q03 = calculateQuaternion(EulerAngle(PI * 1.222, PI * 0.333, PI * 0.777, EulerOrder::XZY));
  auto e03 = toEulerAngle(q03, EulerOrder::XZY);
  auto m03 = calculateRotationMatrix(e03);
  auto q04 = calculateQuaternion(EulerAngle(PI * 0.333, PI * 1.777, 0.5 * PI, EulerOrder::XZY));
  auto e04 = toEulerAngle(q04, EulerOrder::XZY);
  auto m04 = calculateRotationMatrix(e04);
  auto q05 = calculateQuaternion(EulerAngle(-PI * 1.333, -PI * 0.777, -0.5 * PI, EulerOrder::XZY));
  auto e05 = toEulerAngle(q05, EulerOrder::XZY);
  auto m05 = calculateRotationMatrix(e05);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(q00.rotate(v000), m00 * v000));
  EXPECT_TRUE(equals(q01.rotate(v000), m01 * v000));
  EXPECT_TRUE(equals(q02.rotate(v000), m02 * v000));
  EXPECT_TRUE(equals(q03.rotate(v000), m03 * v000));
  EXPECT_TRUE(equals(q04.rotate(v000), m04 * v000));
  EXPECT_TRUE(equals(q05.rotate(v000), m05 * v000));
  EXPECT_TRUE(equals(q00.rotate(v100), m00 * v100));
  EXPECT_TRUE(equals(q01.rotate(v100), m01 * v100));
  EXPECT_TRUE(equals(q02.rotate(v100), m02 * v100));
  EXPECT_TRUE(equals(q03.rotate(v100), m03 * v100));
  EXPECT_TRUE(equals(q04.rotate(v100), m04 * v100));
  EXPECT_TRUE(equals(q05.rotate(v100), m05 * v100));
  EXPECT_TRUE(equals(q00.rotate(v010), m00 * v010));
  EXPECT_TRUE(equals(q01.rotate(v010), m01 * v010));
  EXPECT_TRUE(equals(q02.rotate(v010), m02 * v010));
  EXPECT_TRUE(equals(q03.rotate(v010), m03 * v010));
  EXPECT_TRUE(equals(q04.rotate(v010), m04 * v010));
  EXPECT_TRUE(equals(q05.rotate(v010), m05 * v010));
  EXPECT_TRUE(equals(q00.rotate(v001), m00 * v001));
  EXPECT_TRUE(equals(q01.rotate(v001), m01 * v001));
  EXPECT_TRUE(equals(q02.rotate(v001), m02 * v001));
  EXPECT_TRUE(equals(q03.rotate(v001), m03 * v001));
  EXPECT_TRUE(equals(q04.rotate(v001), m04 * v001));
  EXPECT_TRUE(equals(q05.rotate(v001), m05 * v001));
  EXPECT_TRUE(equals(q00.rotate(va), m00 * va));
  EXPECT_TRUE(equals(q01.rotate(va), m01 * va));
  EXPECT_TRUE(equals(q02.rotate(va), m02 * va));
  EXPECT_TRUE(equals(q03.rotate(va), m03 * va));
  EXPECT_TRUE(equals(q04.rotate(va), m04 * va));
  EXPECT_TRUE(equals(q05.rotate(va), m05 * va));
  EXPECT_TRUE(equals(q00.rotate(vb), m00 * vb));
  EXPECT_TRUE(equals(q01.rotate(vb), m01 * vb));
  EXPECT_TRUE(equals(q02.rotate(vb), m02 * vb));
  EXPECT_TRUE(equals(q03.rotate(vb), m03 * vb));
  EXPECT_TRUE(equals(q04.rotate(vb), m04 * vb));
  EXPECT_TRUE(equals(q05.rotate(vb), m05 * vb));
  EXPECT_TRUE(equals(q00.rotate(vc), m00 * vc));
  EXPECT_TRUE(equals(q01.rotate(vc), m01 * vc));
  EXPECT_TRUE(equals(q02.rotate(vc), m02 * vc));
  EXPECT_TRUE(equals(q03.rotate(vc), m03 * vc));
  EXPECT_TRUE(equals(q04.rotate(vc), m04 * vc));
  EXPECT_TRUE(equals(q05.rotate(vc), m05 * vc));
}

TEST(QuaternionToEulerAngle, YXZ) {
  auto q00 = calculateQuaternion(EulerAngle(0, 0, 0, EulerOrder::YXZ));
  auto e00 = toEulerAngle(q00, EulerOrder::YXZ);
  auto m00 = calculateRotationMatrix(e00);
  auto q01 = calculateQuaternion(EulerAngle(PI * 0.333, PI * 0.777, PI * 1.222, EulerOrder::YXZ));
  auto e01 = toEulerAngle(q01, EulerOrder::YXZ);
  auto m01 = calculateRotationMatrix(e01);
  auto q02 = calculateQuaternion(EulerAngle(PI * 0.777, PI * 1.222, PI * 0.333, EulerOrder::YXZ));
  auto e02 = toEulerAngle(q02, EulerOrder::YXZ);
  auto m02 = calculateRotationMatrix(e02);
  auto q03 = calculateQuaternion(EulerAngle(PI * 1.222, PI * 0.333, PI * 0.777, EulerOrder::YXZ));
  auto e03 = toEulerAngle(q03, EulerOrder::YXZ);
  auto m03 = calculateRotationMatrix(e03);
  auto q04 = calculateQuaternion(EulerAngle(0.5 * PI, PI * 0.333, PI * 1.777, EulerOrder::YXZ));
  auto e04 = toEulerAngle(q04, EulerOrder::YXZ);
  auto m04 = calculateRotationMatrix(e04);
  auto q05 = calculateQuaternion(EulerAngle(-0.5 * PI, -PI * 1.333, -PI * 0.777, EulerOrder::YXZ));
  auto e05 = toEulerAngle(q05, EulerOrder::YXZ);
  auto m05 = calculateRotationMatrix(e05);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(q00.rotate(v000), m00 * v000));
  EXPECT_TRUE(equals(q01.rotate(v000), m01 * v000));
  EXPECT_TRUE(equals(q02.rotate(v000), m02 * v000));
  EXPECT_TRUE(equals(q03.rotate(v000), m03 * v000));
  EXPECT_TRUE(equals(q04.rotate(v000), m04 * v000));
  EXPECT_TRUE(equals(q05.rotate(v000), m05 * v000));
  EXPECT_TRUE(equals(q00.rotate(v100), m00 * v100));
  EXPECT_TRUE(equals(q01.rotate(v100), m01 * v100));
  EXPECT_TRUE(equals(q02.rotate(v100), m02 * v100));
  EXPECT_TRUE(equals(q03.rotate(v100), m03 * v100));
  EXPECT_TRUE(equals(q04.rotate(v100), m04 * v100));
  EXPECT_TRUE(equals(q05.rotate(v100), m05 * v100));
  EXPECT_TRUE(equals(q00.rotate(v010), m00 * v010));
  EXPECT_TRUE(equals(q01.rotate(v010), m01 * v010));
  EXPECT_TRUE(equals(q02.rotate(v010), m02 * v010));
  EXPECT_TRUE(equals(q03.rotate(v010), m03 * v010));
  EXPECT_TRUE(equals(q04.rotate(v010), m04 * v010));
  EXPECT_TRUE(equals(q05.rotate(v010), m05 * v010));
  EXPECT_TRUE(equals(q00.rotate(v001), m00 * v001));
  EXPECT_TRUE(equals(q01.rotate(v001), m01 * v001));
  EXPECT_TRUE(equals(q02.rotate(v001), m02 * v001));
  EXPECT_TRUE(equals(q03.rotate(v001), m03 * v001));
  EXPECT_TRUE(equals(q04.rotate(v001), m04 * v001));
  EXPECT_TRUE(equals(q05.rotate(v001), m05 * v001));
  EXPECT_TRUE(equals(q00.rotate(va), m00 * va));
  EXPECT_TRUE(equals(q01.rotate(va), m01 * va));
  EXPECT_TRUE(equals(q02.rotate(va), m02 * va));
  EXPECT_TRUE(equals(q03.rotate(va), m03 * va));
  EXPECT_TRUE(equals(q04.rotate(va), m04 * va));
  EXPECT_TRUE(equals(q05.rotate(va), m05 * va));
  EXPECT_TRUE(equals(q00.rotate(vb), m00 * vb));
  EXPECT_TRUE(equals(q01.rotate(vb), m01 * vb));
  EXPECT_TRUE(equals(q02.rotate(vb), m02 * vb));
  EXPECT_TRUE(equals(q03.rotate(vb), m03 * vb));
  EXPECT_TRUE(equals(q04.rotate(vb), m04 * vb));
  EXPECT_TRUE(equals(q05.rotate(vb), m05 * vb));
  EXPECT_TRUE(equals(q00.rotate(vc), m00 * vc));
  EXPECT_TRUE(equals(q01.rotate(vc), m01 * vc));
  EXPECT_TRUE(equals(q02.rotate(vc), m02 * vc));
  EXPECT_TRUE(equals(q03.rotate(vc), m03 * vc));
  EXPECT_TRUE(equals(q04.rotate(vc), m04 * vc));
  EXPECT_TRUE(equals(q05.rotate(vc), m05 * vc));
}

TEST(QuaternionToEulerAngle, YZX) {
  auto q00 = calculateQuaternion(EulerAngle(0, 0, 0, EulerOrder::YZX));
  auto e00 = toEulerAngle(q00, EulerOrder::YZX);
  auto m00 = calculateRotationMatrix(e00);
  auto q01 = calculateQuaternion(EulerAngle(PI * 0.333, PI * 0.777, PI * 1.222, EulerOrder::YZX));
  auto e01 = toEulerAngle(q01, EulerOrder::YZX);
  auto m01 = calculateRotationMatrix(e01);
  auto q02 = calculateQuaternion(EulerAngle(PI * 0.777, PI * 1.222, PI * 0.333, EulerOrder::YZX));
  auto e02 = toEulerAngle(q02, EulerOrder::YZX);
  auto m02 = calculateRotationMatrix(e02);
  auto q03 = calculateQuaternion(EulerAngle(PI * 1.222, PI * 0.333, PI * 0.777, EulerOrder::YZX));
  auto e03 = toEulerAngle(q03, EulerOrder::YZX);
  auto m03 = calculateRotationMatrix(e03);
  auto q04 = calculateQuaternion(EulerAngle(PI * 0.333, PI * 1.777, 0.5 * PI, EulerOrder::YZX));
  auto e04 = toEulerAngle(q04, EulerOrder::YZX);
  auto m04 = calculateRotationMatrix(e04);
  auto q05 = calculateQuaternion(EulerAngle(-PI * 1.333, -PI * 0.777, -0.5 * PI, EulerOrder::YZX));
  auto e05 = toEulerAngle(q05, EulerOrder::YZX);
  auto m05 = calculateRotationMatrix(e05);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(q00.rotate(v000), m00 * v000));
  EXPECT_TRUE(equals(q01.rotate(v000), m01 * v000));
  EXPECT_TRUE(equals(q02.rotate(v000), m02 * v000));
  EXPECT_TRUE(equals(q03.rotate(v000), m03 * v000));
  EXPECT_TRUE(equals(q04.rotate(v000), m04 * v000));
  EXPECT_TRUE(equals(q05.rotate(v000), m05 * v000));
  EXPECT_TRUE(equals(q00.rotate(v100), m00 * v100));
  EXPECT_TRUE(equals(q01.rotate(v100), m01 * v100));
  EXPECT_TRUE(equals(q02.rotate(v100), m02 * v100));
  EXPECT_TRUE(equals(q03.rotate(v100), m03 * v100));
  EXPECT_TRUE(equals(q04.rotate(v100), m04 * v100));
  EXPECT_TRUE(equals(q05.rotate(v100), m05 * v100));
  EXPECT_TRUE(equals(q00.rotate(v010), m00 * v010));
  EXPECT_TRUE(equals(q01.rotate(v010), m01 * v010));
  EXPECT_TRUE(equals(q02.rotate(v010), m02 * v010));
  EXPECT_TRUE(equals(q03.rotate(v010), m03 * v010));
  EXPECT_TRUE(equals(q04.rotate(v010), m04 * v010));
  EXPECT_TRUE(equals(q05.rotate(v010), m05 * v010));
  EXPECT_TRUE(equals(q00.rotate(v001), m00 * v001));
  EXPECT_TRUE(equals(q01.rotate(v001), m01 * v001));
  EXPECT_TRUE(equals(q02.rotate(v001), m02 * v001));
  EXPECT_TRUE(equals(q03.rotate(v001), m03 * v001));
  EXPECT_TRUE(equals(q04.rotate(v001), m04 * v001));
  EXPECT_TRUE(equals(q05.rotate(v001), m05 * v001));
  EXPECT_TRUE(equals(q00.rotate(va), m00 * va));
  EXPECT_TRUE(equals(q01.rotate(va), m01 * va));
  EXPECT_TRUE(equals(q02.rotate(va), m02 * va));
  EXPECT_TRUE(equals(q03.rotate(va), m03 * va));
  EXPECT_TRUE(equals(q04.rotate(va), m04 * va));
  EXPECT_TRUE(equals(q05.rotate(va), m05 * va));
  EXPECT_TRUE(equals(q00.rotate(vb), m00 * vb));
  EXPECT_TRUE(equals(q01.rotate(vb), m01 * vb));
  EXPECT_TRUE(equals(q02.rotate(vb), m02 * vb));
  EXPECT_TRUE(equals(q03.rotate(vb), m03 * vb));
  EXPECT_TRUE(equals(q04.rotate(vb), m04 * vb));
  EXPECT_TRUE(equals(q05.rotate(vb), m05 * vb));
  EXPECT_TRUE(equals(q00.rotate(vc), m00 * vc));
  EXPECT_TRUE(equals(q01.rotate(vc), m01 * vc));
  EXPECT_TRUE(equals(q02.rotate(vc), m02 * vc));
  EXPECT_TRUE(equals(q03.rotate(vc), m03 * vc));
  EXPECT_TRUE(equals(q04.rotate(vc), m04 * vc));
  EXPECT_TRUE(equals(q05.rotate(vc), m05 * vc));
}

TEST(QuaternionToEulerAngle, ZXY) {
  auto q00 = calculateQuaternion(EulerAngle(0, 0, 0, EulerOrder::ZXY));
  auto e00 = toEulerAngle(q00, EulerOrder::ZXY);
  auto m00 = calculateRotationMatrix(e00);
  auto q01 = calculateQuaternion(EulerAngle(PI * 0.333, PI * 0.777, PI * 1.222, EulerOrder::ZXY));
  auto e01 = toEulerAngle(q01, EulerOrder::ZXY);
  auto m01 = calculateRotationMatrix(e01);
  auto q02 = calculateQuaternion(EulerAngle(PI * 0.777, PI * 1.222, PI * 0.333, EulerOrder::ZXY));
  auto e02 = toEulerAngle(q02, EulerOrder::ZXY);
  auto m02 = calculateRotationMatrix(e02);
  auto q03 = calculateQuaternion(EulerAngle(PI * 1.222, PI * 0.333, PI * 0.777, EulerOrder::ZXY));
  auto e03 = toEulerAngle(q03, EulerOrder::ZXY);
  auto m03 = calculateRotationMatrix(e03);
  auto q04 = calculateQuaternion(EulerAngle(0.5 * PI, PI * 0.333, PI * 1.777, EulerOrder::ZXY));
  auto e04 = toEulerAngle(q04, EulerOrder::ZXY);
  auto m04 = calculateRotationMatrix(e04);
  auto q05 = calculateQuaternion(EulerAngle(-0.5 * PI, -PI * 1.333, -PI * 0.777, EulerOrder::ZXY));
  auto e05 = toEulerAngle(q05, EulerOrder::ZXY);
  auto m05 = calculateRotationMatrix(e05);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(q00.rotate(v000), m00 * v000));
  EXPECT_TRUE(equals(q01.rotate(v000), m01 * v000));
  EXPECT_TRUE(equals(q02.rotate(v000), m02 * v000));
  EXPECT_TRUE(equals(q03.rotate(v000), m03 * v000));
  EXPECT_TRUE(equals(q04.rotate(v000), m04 * v000));
  EXPECT_TRUE(equals(q05.rotate(v000), m05 * v000));
  EXPECT_TRUE(equals(q00.rotate(v100), m00 * v100));
  EXPECT_TRUE(equals(q01.rotate(v100), m01 * v100));
  EXPECT_TRUE(equals(q02.rotate(v100), m02 * v100));
  EXPECT_TRUE(equals(q03.rotate(v100), m03 * v100));
  EXPECT_TRUE(equals(q04.rotate(v100), m04 * v100));
  EXPECT_TRUE(equals(q05.rotate(v100), m05 * v100));
  EXPECT_TRUE(equals(q00.rotate(v010), m00 * v010));
  EXPECT_TRUE(equals(q01.rotate(v010), m01 * v010));
  EXPECT_TRUE(equals(q02.rotate(v010), m02 * v010));
  EXPECT_TRUE(equals(q03.rotate(v010), m03 * v010));
  EXPECT_TRUE(equals(q04.rotate(v010), m04 * v010));
  EXPECT_TRUE(equals(q05.rotate(v010), m05 * v010));
  EXPECT_TRUE(equals(q00.rotate(v001), m00 * v001));
  EXPECT_TRUE(equals(q01.rotate(v001), m01 * v001));
  EXPECT_TRUE(equals(q02.rotate(v001), m02 * v001));
  EXPECT_TRUE(equals(q03.rotate(v001), m03 * v001));
  EXPECT_TRUE(equals(q04.rotate(v001), m04 * v001));
  EXPECT_TRUE(equals(q05.rotate(v001), m05 * v001));
  EXPECT_TRUE(equals(q00.rotate(va), m00 * va));
  EXPECT_TRUE(equals(q01.rotate(va), m01 * va));
  EXPECT_TRUE(equals(q02.rotate(va), m02 * va));
  EXPECT_TRUE(equals(q03.rotate(va), m03 * va));
  EXPECT_TRUE(equals(q04.rotate(va), m04 * va));
  EXPECT_TRUE(equals(q05.rotate(va), m05 * va));
  EXPECT_TRUE(equals(q00.rotate(vb), m00 * vb));
  EXPECT_TRUE(equals(q01.rotate(vb), m01 * vb));
  EXPECT_TRUE(equals(q02.rotate(vb), m02 * vb));
  EXPECT_TRUE(equals(q03.rotate(vb), m03 * vb));
  EXPECT_TRUE(equals(q04.rotate(vb), m04 * vb));
  EXPECT_TRUE(equals(q05.rotate(vb), m05 * vb));
  EXPECT_TRUE(equals(q00.rotate(vc), m00 * vc));
  EXPECT_TRUE(equals(q01.rotate(vc), m01 * vc));
  EXPECT_TRUE(equals(q02.rotate(vc), m02 * vc));
  EXPECT_TRUE(equals(q03.rotate(vc), m03 * vc));
  EXPECT_TRUE(equals(q04.rotate(vc), m04 * vc));
  EXPECT_TRUE(equals(q05.rotate(vc), m05 * vc));
}

TEST(QuaternionToEulerAngle, ZYX) {
  auto q00 = calculateQuaternion(EulerAngle(0, 0, 0, EulerOrder::ZYX));
  auto e00 = toEulerAngle(q00, EulerOrder::ZYX);
  auto m00 = calculateRotationMatrix(e00);
  auto q01 = calculateQuaternion(EulerAngle(PI * 0.333, PI * 0.777, PI * 1.222, EulerOrder::ZYX));
  auto e01 = toEulerAngle(q01, EulerOrder::ZYX);
  auto m01 = calculateRotationMatrix(e01);
  auto q02 = calculateQuaternion(EulerAngle(PI * 0.777, PI * 1.222, PI * 0.333, EulerOrder::ZYX));
  auto e02 = toEulerAngle(q02, EulerOrder::ZYX);
  auto m02 = calculateRotationMatrix(e02);
  auto q03 = calculateQuaternion(EulerAngle(PI * 1.222, PI * 0.333, PI * 0.777, EulerOrder::ZYX));
  auto e03 = toEulerAngle(q03, EulerOrder::ZYX);
  auto m03 = calculateRotationMatrix(e03);
  auto q04 = calculateQuaternion(EulerAngle(PI * 0.333, 0.5 * PI, PI * 1.777, EulerOrder::ZYX));
  auto e04 = toEulerAngle(q04, EulerOrder::ZYX);
  auto m04 = calculateRotationMatrix(e04);
  auto q05 = calculateQuaternion(EulerAngle(-PI * 1.333, -0.5 * PI, -PI * 0.777, EulerOrder::ZYX));
  auto e05 = toEulerAngle(q05, EulerOrder::ZYX);
  auto m05 = calculateRotationMatrix(e05);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(q00.rotate(v000), m00 * v000));
  EXPECT_TRUE(equals(q01.rotate(v000), m01 * v000));
  EXPECT_TRUE(equals(q02.rotate(v000), m02 * v000));
  EXPECT_TRUE(equals(q03.rotate(v000), m03 * v000));
  EXPECT_TRUE(equals(q04.rotate(v000), m04 * v000));
  EXPECT_TRUE(equals(q05.rotate(v000), m05 * v000));
  EXPECT_TRUE(equals(q00.rotate(v100), m00 * v100));
  EXPECT_TRUE(equals(q01.rotate(v100), m01 * v100));
  EXPECT_TRUE(equals(q02.rotate(v100), m02 * v100));
  EXPECT_TRUE(equals(q03.rotate(v100), m03 * v100));
  EXPECT_TRUE(equals(q04.rotate(v100), m04 * v100));
  EXPECT_TRUE(equals(q05.rotate(v100), m05 * v100));
  EXPECT_TRUE(equals(q00.rotate(v010), m00 * v010));
  EXPECT_TRUE(equals(q01.rotate(v010), m01 * v010));
  EXPECT_TRUE(equals(q02.rotate(v010), m02 * v010));
  EXPECT_TRUE(equals(q03.rotate(v010), m03 * v010));
  EXPECT_TRUE(equals(q04.rotate(v010), m04 * v010));
  EXPECT_TRUE(equals(q05.rotate(v010), m05 * v010));
  EXPECT_TRUE(equals(q00.rotate(v001), m00 * v001));
  EXPECT_TRUE(equals(q01.rotate(v001), m01 * v001));
  EXPECT_TRUE(equals(q02.rotate(v001), m02 * v001));
  EXPECT_TRUE(equals(q03.rotate(v001), m03 * v001));
  EXPECT_TRUE(equals(q04.rotate(v001), m04 * v001));
  EXPECT_TRUE(equals(q05.rotate(v001), m05 * v001));
  EXPECT_TRUE(equals(q00.rotate(va), m00 * va));
  EXPECT_TRUE(equals(q01.rotate(va), m01 * va));
  EXPECT_TRUE(equals(q02.rotate(va), m02 * va));
  EXPECT_TRUE(equals(q03.rotate(va), m03 * va));
  EXPECT_TRUE(equals(q04.rotate(va), m04 * va));
  EXPECT_TRUE(equals(q05.rotate(va), m05 * va));
  EXPECT_TRUE(equals(q00.rotate(vb), m00 * vb));
  EXPECT_TRUE(equals(q01.rotate(vb), m01 * vb));
  EXPECT_TRUE(equals(q02.rotate(vb), m02 * vb));
  EXPECT_TRUE(equals(q03.rotate(vb), m03 * vb));
  EXPECT_TRUE(equals(q04.rotate(vb), m04 * vb));
  EXPECT_TRUE(equals(q05.rotate(vb), m05 * vb));
  EXPECT_TRUE(equals(q00.rotate(vc), m00 * vc));
  EXPECT_TRUE(equals(q01.rotate(vc), m01 * vc));
  EXPECT_TRUE(equals(q02.rotate(vc), m02 * vc));
  EXPECT_TRUE(equals(q03.rotate(vc), m03 * vc));
  EXPECT_TRUE(equals(q04.rotate(vc), m04 * vc));
  EXPECT_TRUE(equals(q05.rotate(vc), m05 * vc));
}

TEST(RotationMatrixToEulerAngle, XYZ) {
  auto ma00 = calculateRotationMatrix(EulerAngle(0, 0, 0, EulerOrder::XYZ));
  auto e00 = toEulerAngle(ma00, EulerOrder::XYZ);
  auto mb00 = calculateRotationMatrix(e00);
  auto ma01 = calculateRotationMatrix(EulerAngle(PI * 0.333, PI * 0.777, PI * 1.222, EulerOrder::XYZ));
  auto e01 = toEulerAngle(ma01, EulerOrder::XYZ);
  auto mb01 = calculateRotationMatrix(e01);
  auto ma02 = calculateRotationMatrix(EulerAngle(PI * 0.777, PI * 1.222, PI * 0.333, EulerOrder::XYZ));
  auto e02 = toEulerAngle(ma02, EulerOrder::XYZ);
  auto mb02 = calculateRotationMatrix(e02);
  auto ma03 = calculateRotationMatrix(EulerAngle(PI * 1.222, PI * 0.333, PI * 0.777, EulerOrder::XYZ));
  auto e03 = toEulerAngle(ma03, EulerOrder::XYZ);
  auto mb03 = calculateRotationMatrix(e03);
  auto ma04 = calculateRotationMatrix(EulerAngle(PI * 0.333, 0.5 * PI, PI * 1.777, EulerOrder::XYZ));
  auto e04 = toEulerAngle(ma04, EulerOrder::XYZ);
  auto mb04 = calculateRotationMatrix(e04);
  auto ma05 = calculateRotationMatrix(EulerAngle(-PI * 1.333, -0.5 * PI, -PI * 0.777, EulerOrder::XYZ));
  auto e05 = toEulerAngle(ma05, EulerOrder::XYZ);
  auto mb05 = calculateRotationMatrix(e05);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(ma00 * v000, mb00 * v000));
  EXPECT_TRUE(equals(ma01 * v000, mb01 * v000));
  EXPECT_TRUE(equals(ma02 * v000, mb02 * v000));
  EXPECT_TRUE(equals(ma03 * v000, mb03 * v000));
  EXPECT_TRUE(equals(ma04 * v000, mb04 * v000));
  EXPECT_TRUE(equals(ma05 * v000, mb05 * v000));
  EXPECT_TRUE(equals(ma00 * v100, mb00 * v100));
  EXPECT_TRUE(equals(ma01 * v100, mb01 * v100));
  EXPECT_TRUE(equals(ma02 * v100, mb02 * v100));
  EXPECT_TRUE(equals(ma03 * v100, mb03 * v100));
  EXPECT_TRUE(equals(ma04 * v100, mb04 * v100));
  EXPECT_TRUE(equals(ma05 * v100, mb05 * v100));
  EXPECT_TRUE(equals(ma00 * v010, mb00 * v010));
  EXPECT_TRUE(equals(ma01 * v010, mb01 * v010));
  EXPECT_TRUE(equals(ma02 * v010, mb02 * v010));
  EXPECT_TRUE(equals(ma03 * v010, mb03 * v010));
  EXPECT_TRUE(equals(ma04 * v010, mb04 * v010));
  EXPECT_TRUE(equals(ma05 * v010, mb05 * v010));
  EXPECT_TRUE(equals(ma00 * v001, mb00 * v001));
  EXPECT_TRUE(equals(ma01 * v001, mb01 * v001));
  EXPECT_TRUE(equals(ma02 * v001, mb02 * v001));
  EXPECT_TRUE(equals(ma03 * v001, mb03 * v001));
  EXPECT_TRUE(equals(ma04 * v001, mb04 * v001));
  EXPECT_TRUE(equals(ma05 * v001, mb05 * v001));
  EXPECT_TRUE(equals(ma00 * va, mb00 * va));
  EXPECT_TRUE(equals(ma01 * va, mb01 * va));
  EXPECT_TRUE(equals(ma02 * va, mb02 * va));
  EXPECT_TRUE(equals(ma03 * va, mb03 * va));
  EXPECT_TRUE(equals(ma04 * va, mb04 * va));
  EXPECT_TRUE(equals(ma05 * va, mb05 * va));
  EXPECT_TRUE(equals(ma00 * vb, mb00 * vb));
  EXPECT_TRUE(equals(ma01 * vb, mb01 * vb));
  EXPECT_TRUE(equals(ma02 * vb, mb02 * vb));
  EXPECT_TRUE(equals(ma03 * vb, mb03 * vb));
  EXPECT_TRUE(equals(ma04 * vb, mb04 * vb));
  EXPECT_TRUE(equals(ma05 * vb, mb05 * vb));
  EXPECT_TRUE(equals(ma00 * vc, mb00 * vc));
  EXPECT_TRUE(equals(ma01 * vc, mb01 * vc));
  EXPECT_TRUE(equals(ma02 * vc, mb02 * vc));
  EXPECT_TRUE(equals(ma03 * vc, mb03 * vc));
  EXPECT_TRUE(equals(ma04 * vc, mb04 * vc));
  EXPECT_TRUE(equals(ma05 * vc, mb05 * vc));
}

TEST(RotationMatrixToEulerAngle, XZY) {
  auto ma00 = calculateRotationMatrix(EulerAngle(0, 0, 0, EulerOrder::XZY));
  auto e00 = toEulerAngle(ma00, EulerOrder::XZY);
  auto mb00 = calculateRotationMatrix(e00);
  auto ma01 = calculateRotationMatrix(EulerAngle(PI * 0.333, PI * 0.777, PI * 1.222, EulerOrder::XZY));
  auto e01 = toEulerAngle(ma01, EulerOrder::XZY);
  auto mb01 = calculateRotationMatrix(e01);
  auto ma02 = calculateRotationMatrix(EulerAngle(PI * 0.777, PI * 1.222, PI * 0.333, EulerOrder::XZY));
  auto e02 = toEulerAngle(ma02, EulerOrder::XZY);
  auto mb02 = calculateRotationMatrix(e02);
  auto ma03 = calculateRotationMatrix(EulerAngle(PI * 1.222, PI * 0.333, PI * 0.777, EulerOrder::XZY));
  auto e03 = toEulerAngle(ma03, EulerOrder::XZY);
  auto mb03 = calculateRotationMatrix(e03);
  auto ma04 = calculateRotationMatrix(EulerAngle(PI * 0.333, PI * 1.777, 0.5 * PI, EulerOrder::XZY));
  auto e04 = toEulerAngle(ma04, EulerOrder::XZY);
  auto mb04 = calculateRotationMatrix(e04);
  auto ma05 = calculateRotationMatrix(EulerAngle(-PI * 1.333, -PI * 0.777, -0.5 * PI, EulerOrder::XZY));
  auto e05 = toEulerAngle(ma05, EulerOrder::XZY);
  auto mb05 = calculateRotationMatrix(e05);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(ma00 * v000, mb00 * v000));
  EXPECT_TRUE(equals(ma01 * v000, mb01 * v000));
  EXPECT_TRUE(equals(ma02 * v000, mb02 * v000));
  EXPECT_TRUE(equals(ma03 * v000, mb03 * v000));
  EXPECT_TRUE(equals(ma04 * v000, mb04 * v000));
  EXPECT_TRUE(equals(ma05 * v000, mb05 * v000));
  EXPECT_TRUE(equals(ma00 * v100, mb00 * v100));
  EXPECT_TRUE(equals(ma01 * v100, mb01 * v100));
  EXPECT_TRUE(equals(ma02 * v100, mb02 * v100));
  EXPECT_TRUE(equals(ma03 * v100, mb03 * v100));
  EXPECT_TRUE(equals(ma04 * v100, mb04 * v100));
  EXPECT_TRUE(equals(ma05 * v100, mb05 * v100));
  EXPECT_TRUE(equals(ma00 * v010, mb00 * v010));
  EXPECT_TRUE(equals(ma01 * v010, mb01 * v010));
  EXPECT_TRUE(equals(ma02 * v010, mb02 * v010));
  EXPECT_TRUE(equals(ma03 * v010, mb03 * v010));
  EXPECT_TRUE(equals(ma04 * v010, mb04 * v010));
  EXPECT_TRUE(equals(ma05 * v010, mb05 * v010));
  EXPECT_TRUE(equals(ma00 * v001, mb00 * v001));
  EXPECT_TRUE(equals(ma01 * v001, mb01 * v001));
  EXPECT_TRUE(equals(ma02 * v001, mb02 * v001));
  EXPECT_TRUE(equals(ma03 * v001, mb03 * v001));
  EXPECT_TRUE(equals(ma04 * v001, mb04 * v001));
  EXPECT_TRUE(equals(ma05 * v001, mb05 * v001));
  EXPECT_TRUE(equals(ma00 * va, mb00 * va));
  EXPECT_TRUE(equals(ma01 * va, mb01 * va));
  EXPECT_TRUE(equals(ma02 * va, mb02 * va));
  EXPECT_TRUE(equals(ma03 * va, mb03 * va));
  EXPECT_TRUE(equals(ma04 * va, mb04 * va));
  EXPECT_TRUE(equals(ma05 * va, mb05 * va));
  EXPECT_TRUE(equals(ma00 * vb, mb00 * vb));
  EXPECT_TRUE(equals(ma01 * vb, mb01 * vb));
  EXPECT_TRUE(equals(ma02 * vb, mb02 * vb));
  EXPECT_TRUE(equals(ma03 * vb, mb03 * vb));
  EXPECT_TRUE(equals(ma04 * vb, mb04 * vb));
  EXPECT_TRUE(equals(ma05 * vb, mb05 * vb));
  EXPECT_TRUE(equals(ma00 * vc, mb00 * vc));
  EXPECT_TRUE(equals(ma01 * vc, mb01 * vc));
  EXPECT_TRUE(equals(ma02 * vc, mb02 * vc));
  EXPECT_TRUE(equals(ma03 * vc, mb03 * vc));
  EXPECT_TRUE(equals(ma04 * vc, mb04 * vc));
  EXPECT_TRUE(equals(ma05 * vc, mb05 * vc));
}

TEST(RotationMatrixToEulerAngle, YXZ) {
  auto ma00 = calculateRotationMatrix(EulerAngle(0, 0, 0, EulerOrder::YXZ));
  auto e00 = toEulerAngle(ma00, EulerOrder::YXZ);
  auto mb00 = calculateRotationMatrix(e00);
  auto ma01 = calculateRotationMatrix(EulerAngle(PI * 0.333, PI * 0.777, PI * 1.222, EulerOrder::YXZ));
  auto e01 = toEulerAngle(ma01, EulerOrder::YXZ);
  auto mb01 = calculateRotationMatrix(e01);
  auto ma02 = calculateRotationMatrix(EulerAngle(PI * 0.777, PI * 1.222, PI * 0.333, EulerOrder::YXZ));
  auto e02 = toEulerAngle(ma02, EulerOrder::YXZ);
  auto mb02 = calculateRotationMatrix(e02);
  auto ma03 = calculateRotationMatrix(EulerAngle(PI * 1.222, PI * 0.333, PI * 0.777, EulerOrder::YXZ));
  auto e03 = toEulerAngle(ma03, EulerOrder::YXZ);
  auto mb03 = calculateRotationMatrix(e03);
  auto ma04 = calculateRotationMatrix(EulerAngle(0.5 * PI, PI * 0.333, PI * 1.777, EulerOrder::YXZ));
  auto e04 = toEulerAngle(ma04, EulerOrder::YXZ);
  auto mb04 = calculateRotationMatrix(e04);
  auto ma05 = calculateRotationMatrix(EulerAngle(-0.5 * PI, -PI * 1.333, -PI * 0.777, EulerOrder::YXZ));
  auto e05 = toEulerAngle(ma05, EulerOrder::YXZ);
  auto mb05 = calculateRotationMatrix(e05);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(ma00 * v000, mb00 * v000));
  EXPECT_TRUE(equals(ma01 * v000, mb01 * v000));
  EXPECT_TRUE(equals(ma02 * v000, mb02 * v000));
  EXPECT_TRUE(equals(ma03 * v000, mb03 * v000));
  EXPECT_TRUE(equals(ma04 * v000, mb04 * v000));
  EXPECT_TRUE(equals(ma05 * v000, mb05 * v000));
  EXPECT_TRUE(equals(ma00 * v100, mb00 * v100));
  EXPECT_TRUE(equals(ma01 * v100, mb01 * v100));
  EXPECT_TRUE(equals(ma02 * v100, mb02 * v100));
  EXPECT_TRUE(equals(ma03 * v100, mb03 * v100));
  EXPECT_TRUE(equals(ma04 * v100, mb04 * v100));
  EXPECT_TRUE(equals(ma05 * v100, mb05 * v100));
  EXPECT_TRUE(equals(ma00 * v010, mb00 * v010));
  EXPECT_TRUE(equals(ma01 * v010, mb01 * v010));
  EXPECT_TRUE(equals(ma02 * v010, mb02 * v010));
  EXPECT_TRUE(equals(ma03 * v010, mb03 * v010));
  EXPECT_TRUE(equals(ma04 * v010, mb04 * v010));
  EXPECT_TRUE(equals(ma05 * v010, mb05 * v010));
  EXPECT_TRUE(equals(ma00 * v001, mb00 * v001));
  EXPECT_TRUE(equals(ma01 * v001, mb01 * v001));
  EXPECT_TRUE(equals(ma02 * v001, mb02 * v001));
  EXPECT_TRUE(equals(ma03 * v001, mb03 * v001));
  EXPECT_TRUE(equals(ma04 * v001, mb04 * v001));
  EXPECT_TRUE(equals(ma05 * v001, mb05 * v001));
  EXPECT_TRUE(equals(ma00 * va, mb00 * va));
  EXPECT_TRUE(equals(ma01 * va, mb01 * va));
  EXPECT_TRUE(equals(ma02 * va, mb02 * va));
  EXPECT_TRUE(equals(ma03 * va, mb03 * va));
  EXPECT_TRUE(equals(ma04 * va, mb04 * va));
  EXPECT_TRUE(equals(ma05 * va, mb05 * va));
  EXPECT_TRUE(equals(ma00 * vb, mb00 * vb));
  EXPECT_TRUE(equals(ma01 * vb, mb01 * vb));
  EXPECT_TRUE(equals(ma02 * vb, mb02 * vb));
  EXPECT_TRUE(equals(ma03 * vb, mb03 * vb));
  EXPECT_TRUE(equals(ma04 * vb, mb04 * vb));
  EXPECT_TRUE(equals(ma05 * vb, mb05 * vb));
  EXPECT_TRUE(equals(ma00 * vc, mb00 * vc));
  EXPECT_TRUE(equals(ma01 * vc, mb01 * vc));
  EXPECT_TRUE(equals(ma02 * vc, mb02 * vc));
  EXPECT_TRUE(equals(ma03 * vc, mb03 * vc));
  EXPECT_TRUE(equals(ma04 * vc, mb04 * vc));
  EXPECT_TRUE(equals(ma05 * vc, mb05 * vc));
}

TEST(RotationMatrixToEulerAngle, YZX) {
  auto ma00 = calculateRotationMatrix(EulerAngle(0, 0, 0, EulerOrder::YZX));
  auto e00 = toEulerAngle(ma00, EulerOrder::YZX);
  auto mb00 = calculateRotationMatrix(e00);
  auto ma01 = calculateRotationMatrix(EulerAngle(PI * 0.333, PI * 0.777, PI * 1.222, EulerOrder::YZX));
  auto e01 = toEulerAngle(ma01, EulerOrder::YZX);
  auto mb01 = calculateRotationMatrix(e01);
  auto ma02 = calculateRotationMatrix(EulerAngle(PI * 0.777, PI * 1.222, PI * 0.333, EulerOrder::YZX));
  auto e02 = toEulerAngle(ma02, EulerOrder::YZX);
  auto mb02 = calculateRotationMatrix(e02);
  auto ma03 = calculateRotationMatrix(EulerAngle(PI * 1.222, PI * 0.333, PI * 0.777, EulerOrder::YZX));
  auto e03 = toEulerAngle(ma03, EulerOrder::YZX);
  auto mb03 = calculateRotationMatrix(e03);
  auto ma04 = calculateRotationMatrix(EulerAngle(0.5 * PI, PI * 0.333, PI * 1.777, EulerOrder::YZX));
  auto e04 = toEulerAngle(ma04, EulerOrder::YZX);
  auto mb04 = calculateRotationMatrix(e04);
  auto ma05 = calculateRotationMatrix(EulerAngle(-0.5 * PI, -PI * 1.333, -PI * 0.777, EulerOrder::YZX));
  auto e05 = toEulerAngle(ma05, EulerOrder::YZX);
  auto mb05 = calculateRotationMatrix(e05);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(ma00 * v000, mb00 * v000));
  EXPECT_TRUE(equals(ma01 * v000, mb01 * v000));
  EXPECT_TRUE(equals(ma02 * v000, mb02 * v000));
  EXPECT_TRUE(equals(ma03 * v000, mb03 * v000));
  EXPECT_TRUE(equals(ma04 * v000, mb04 * v000));
  EXPECT_TRUE(equals(ma05 * v000, mb05 * v000));
  EXPECT_TRUE(equals(ma00 * v100, mb00 * v100));
  EXPECT_TRUE(equals(ma01 * v100, mb01 * v100));
  EXPECT_TRUE(equals(ma02 * v100, mb02 * v100));
  EXPECT_TRUE(equals(ma03 * v100, mb03 * v100));
  EXPECT_TRUE(equals(ma04 * v100, mb04 * v100));
  EXPECT_TRUE(equals(ma05 * v100, mb05 * v100));
  EXPECT_TRUE(equals(ma00 * v010, mb00 * v010));
  EXPECT_TRUE(equals(ma01 * v010, mb01 * v010));
  EXPECT_TRUE(equals(ma02 * v010, mb02 * v010));
  EXPECT_TRUE(equals(ma03 * v010, mb03 * v010));
  EXPECT_TRUE(equals(ma04 * v010, mb04 * v010));
  EXPECT_TRUE(equals(ma05 * v010, mb05 * v010));
  EXPECT_TRUE(equals(ma00 * v001, mb00 * v001));
  EXPECT_TRUE(equals(ma01 * v001, mb01 * v001));
  EXPECT_TRUE(equals(ma02 * v001, mb02 * v001));
  EXPECT_TRUE(equals(ma03 * v001, mb03 * v001));
  EXPECT_TRUE(equals(ma04 * v001, mb04 * v001));
  EXPECT_TRUE(equals(ma05 * v001, mb05 * v001));
  EXPECT_TRUE(equals(ma00 * va, mb00 * va));
  EXPECT_TRUE(equals(ma01 * va, mb01 * va));
  EXPECT_TRUE(equals(ma02 * va, mb02 * va));
  EXPECT_TRUE(equals(ma03 * va, mb03 * va));
  EXPECT_TRUE(equals(ma04 * va, mb04 * va));
  EXPECT_TRUE(equals(ma05 * va, mb05 * va));
  EXPECT_TRUE(equals(ma00 * vb, mb00 * vb));
  EXPECT_TRUE(equals(ma01 * vb, mb01 * vb));
  EXPECT_TRUE(equals(ma02 * vb, mb02 * vb));
  EXPECT_TRUE(equals(ma03 * vb, mb03 * vb));
  EXPECT_TRUE(equals(ma04 * vb, mb04 * vb));
  EXPECT_TRUE(equals(ma05 * vb, mb05 * vb));
  EXPECT_TRUE(equals(ma00 * vc, mb00 * vc));
  EXPECT_TRUE(equals(ma01 * vc, mb01 * vc));
  EXPECT_TRUE(equals(ma02 * vc, mb02 * vc));
  EXPECT_TRUE(equals(ma03 * vc, mb03 * vc));
  EXPECT_TRUE(equals(ma04 * vc, mb04 * vc));
  EXPECT_TRUE(equals(ma05 * vc, mb05 * vc));
}

TEST(RotationMatrixToEulerAngle, ZXY) {
  auto ma00 = calculateRotationMatrix(EulerAngle(0, 0, 0, EulerOrder::ZXY));
  auto e00 = toEulerAngle(ma00, EulerOrder::ZXY);
  auto mb00 = calculateRotationMatrix(e00);
  auto ma01 = calculateRotationMatrix(EulerAngle(PI * 0.333, PI * 0.777, PI * 1.222, EulerOrder::ZXY));
  auto e01 = toEulerAngle(ma01, EulerOrder::ZXY);
  auto mb01 = calculateRotationMatrix(e01);
  auto ma02 = calculateRotationMatrix(EulerAngle(PI * 0.777, PI * 1.222, PI * 0.333, EulerOrder::ZXY));
  auto e02 = toEulerAngle(ma02, EulerOrder::ZXY);
  auto mb02 = calculateRotationMatrix(e02);
  auto ma03 = calculateRotationMatrix(EulerAngle(PI * 1.222, PI * 0.333, PI * 0.777, EulerOrder::ZXY));
  auto e03 = toEulerAngle(ma03, EulerOrder::ZXY);
  auto mb03 = calculateRotationMatrix(e03);
  auto ma04 = calculateRotationMatrix(EulerAngle(PI * 0.333, 0.5 * PI, PI * 1.777, EulerOrder::ZXY));
  auto e04 = toEulerAngle(ma04, EulerOrder::ZXY);
  auto mb04 = calculateRotationMatrix(e04);
  auto ma05 = calculateRotationMatrix(EulerAngle(-PI * 1.333, -0.5 * PI, -PI * 0.777, EulerOrder::ZXY));
  auto e05 = toEulerAngle(ma05, EulerOrder::ZXY);
  auto mb05 = calculateRotationMatrix(e05);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(ma00 * v000, mb00 * v000));
  EXPECT_TRUE(equals(ma01 * v000, mb01 * v000));
  EXPECT_TRUE(equals(ma02 * v000, mb02 * v000));
  EXPECT_TRUE(equals(ma03 * v000, mb03 * v000));
  EXPECT_TRUE(equals(ma04 * v000, mb04 * v000));
  EXPECT_TRUE(equals(ma05 * v000, mb05 * v000));
  EXPECT_TRUE(equals(ma00 * v100, mb00 * v100));
  EXPECT_TRUE(equals(ma01 * v100, mb01 * v100));
  EXPECT_TRUE(equals(ma02 * v100, mb02 * v100));
  EXPECT_TRUE(equals(ma03 * v100, mb03 * v100));
  EXPECT_TRUE(equals(ma04 * v100, mb04 * v100));
  EXPECT_TRUE(equals(ma05 * v100, mb05 * v100));
  EXPECT_TRUE(equals(ma00 * v010, mb00 * v010));
  EXPECT_TRUE(equals(ma01 * v010, mb01 * v010));
  EXPECT_TRUE(equals(ma02 * v010, mb02 * v010));
  EXPECT_TRUE(equals(ma03 * v010, mb03 * v010));
  EXPECT_TRUE(equals(ma04 * v010, mb04 * v010));
  EXPECT_TRUE(equals(ma05 * v010, mb05 * v010));
  EXPECT_TRUE(equals(ma00 * v001, mb00 * v001));
  EXPECT_TRUE(equals(ma01 * v001, mb01 * v001));
  EXPECT_TRUE(equals(ma02 * v001, mb02 * v001));
  EXPECT_TRUE(equals(ma03 * v001, mb03 * v001));
  EXPECT_TRUE(equals(ma04 * v001, mb04 * v001));
  EXPECT_TRUE(equals(ma05 * v001, mb05 * v001));
  EXPECT_TRUE(equals(ma00 * va, mb00 * va));
  EXPECT_TRUE(equals(ma01 * va, mb01 * va));
  EXPECT_TRUE(equals(ma02 * va, mb02 * va));
  EXPECT_TRUE(equals(ma03 * va, mb03 * va));
  EXPECT_TRUE(equals(ma04 * va, mb04 * va));
  EXPECT_TRUE(equals(ma05 * va, mb05 * va));
  EXPECT_TRUE(equals(ma00 * vb, mb00 * vb));
  EXPECT_TRUE(equals(ma01 * vb, mb01 * vb));
  EXPECT_TRUE(equals(ma02 * vb, mb02 * vb));
  EXPECT_TRUE(equals(ma03 * vb, mb03 * vb));
  EXPECT_TRUE(equals(ma04 * vb, mb04 * vb));
  EXPECT_TRUE(equals(ma05 * vb, mb05 * vb));
  EXPECT_TRUE(equals(ma00 * vc, mb00 * vc));
  EXPECT_TRUE(equals(ma01 * vc, mb01 * vc));
  EXPECT_TRUE(equals(ma02 * vc, mb02 * vc));
  EXPECT_TRUE(equals(ma03 * vc, mb03 * vc));
  EXPECT_TRUE(equals(ma04 * vc, mb04 * vc));
  EXPECT_TRUE(equals(ma05 * vc, mb05 * vc));
}

TEST(RotationMatrixToEulerAngle, ZYX) {
  auto ma00 = calculateRotationMatrix(EulerAngle(0, 0, 0, EulerOrder::ZYX));
  auto e00 = toEulerAngle(ma00, EulerOrder::ZYX);
  auto mb00 = calculateRotationMatrix(e00);
  auto ma01 = calculateRotationMatrix(EulerAngle(PI * 0.333, PI * 0.777, PI * 1.222, EulerOrder::ZYX));
  auto e01 = toEulerAngle(ma01, EulerOrder::ZYX);
  auto mb01 = calculateRotationMatrix(e01);
  auto ma02 = calculateRotationMatrix(EulerAngle(PI * 0.777, PI * 1.222, PI * 0.333, EulerOrder::ZYX));
  auto e02 = toEulerAngle(ma02, EulerOrder::ZYX);
  auto mb02 = calculateRotationMatrix(e02);
  auto ma03 = calculateRotationMatrix(EulerAngle(PI * 1.222, PI * 0.333, PI * 0.777, EulerOrder::ZYX));
  auto e03 = toEulerAngle(ma03, EulerOrder::ZYX);
  auto mb03 = calculateRotationMatrix(e03);
  auto ma04 = calculateRotationMatrix(EulerAngle(PI * 0.333, 0.5 * PI, PI * 1.777, EulerOrder::ZYX));
  auto e04 = toEulerAngle(ma04, EulerOrder::ZYX);
  auto mb04 = calculateRotationMatrix(e04);
  auto ma05 = calculateRotationMatrix(EulerAngle(-PI * 1.333, -0.5 * PI, -PI * 0.777, EulerOrder::ZYX));
  auto e05 = toEulerAngle(ma05, EulerOrder::ZYX);
  auto mb05 = calculateRotationMatrix(e05);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(ma00 * v000, mb00 * v000));
  EXPECT_TRUE(equals(ma01 * v000, mb01 * v000));
  EXPECT_TRUE(equals(ma02 * v000, mb02 * v000));
  EXPECT_TRUE(equals(ma03 * v000, mb03 * v000));
  EXPECT_TRUE(equals(ma04 * v000, mb04 * v000));
  EXPECT_TRUE(equals(ma05 * v000, mb05 * v000));
  EXPECT_TRUE(equals(ma00 * v100, mb00 * v100));
  EXPECT_TRUE(equals(ma01 * v100, mb01 * v100));
  EXPECT_TRUE(equals(ma02 * v100, mb02 * v100));
  EXPECT_TRUE(equals(ma03 * v100, mb03 * v100));
  EXPECT_TRUE(equals(ma04 * v100, mb04 * v100));
  EXPECT_TRUE(equals(ma05 * v100, mb05 * v100));
  EXPECT_TRUE(equals(ma00 * v010, mb00 * v010));
  EXPECT_TRUE(equals(ma01 * v010, mb01 * v010));
  EXPECT_TRUE(equals(ma02 * v010, mb02 * v010));
  EXPECT_TRUE(equals(ma03 * v010, mb03 * v010));
  EXPECT_TRUE(equals(ma04 * v010, mb04 * v010));
  EXPECT_TRUE(equals(ma05 * v010, mb05 * v010));
  EXPECT_TRUE(equals(ma00 * v001, mb00 * v001));
  EXPECT_TRUE(equals(ma01 * v001, mb01 * v001));
  EXPECT_TRUE(equals(ma02 * v001, mb02 * v001));
  EXPECT_TRUE(equals(ma03 * v001, mb03 * v001));
  EXPECT_TRUE(equals(ma04 * v001, mb04 * v001));
  EXPECT_TRUE(equals(ma05 * v001, mb05 * v001));
  EXPECT_TRUE(equals(ma00 * va, mb00 * va));
  EXPECT_TRUE(equals(ma01 * va, mb01 * va));
  EXPECT_TRUE(equals(ma02 * va, mb02 * va));
  EXPECT_TRUE(equals(ma03 * va, mb03 * va));
  EXPECT_TRUE(equals(ma04 * va, mb04 * va));
  EXPECT_TRUE(equals(ma05 * va, mb05 * va));
  EXPECT_TRUE(equals(ma00 * vb, mb00 * vb));
  EXPECT_TRUE(equals(ma01 * vb, mb01 * vb));
  EXPECT_TRUE(equals(ma02 * vb, mb02 * vb));
  EXPECT_TRUE(equals(ma03 * vb, mb03 * vb));
  EXPECT_TRUE(equals(ma04 * vb, mb04 * vb));
  EXPECT_TRUE(equals(ma05 * vb, mb05 * vb));
  EXPECT_TRUE(equals(ma00 * vc, mb00 * vc));
  EXPECT_TRUE(equals(ma01 * vc, mb01 * vc));
  EXPECT_TRUE(equals(ma02 * vc, mb02 * vc));
  EXPECT_TRUE(equals(ma03 * vc, mb03 * vc));
  EXPECT_TRUE(equals(ma04 * vc, mb04 * vc));
  EXPECT_TRUE(equals(ma05 * vc, mb05 * vc));
}

TEST(EulerAngleToRotationMatrix, XYZ) {
  auto e00 = EulerAngle(0, 0, 0, EulerOrder::XYZ);
  auto ma00 = toRotationMatrix(e00);
  auto mb00 = calculateRotationMatrix(e00);
  auto e01 = EulerAngle(PI * 0.333, PI * 0.777, PI * 1.222, EulerOrder::XYZ);
  auto ma01 = toRotationMatrix(e01);
  auto mb01 = calculateRotationMatrix(e01);
  auto e02 = EulerAngle(PI * 0.777, PI * 1.222, PI * 0.333, EulerOrder::XYZ);
  auto ma02 = toRotationMatrix(e02);
  auto mb02 = calculateRotationMatrix(e02);
  auto e03 = EulerAngle(PI * 1.222, PI * 0.333, PI * 0.777, EulerOrder::XYZ);
  auto ma03 = toRotationMatrix(e03);
  auto mb03 = calculateRotationMatrix(e03);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(ma00 * v000, mb00 * v000));
  EXPECT_TRUE(equals(ma01 * v000, mb01 * v000));
  EXPECT_TRUE(equals(ma02 * v000, mb02 * v000));
  EXPECT_TRUE(equals(ma03 * v000, mb03 * v000));
  EXPECT_TRUE(equals(ma00 * v100, mb00 * v100));
  EXPECT_TRUE(equals(ma01 * v100, mb01 * v100));
  EXPECT_TRUE(equals(ma02 * v100, mb02 * v100));
  EXPECT_TRUE(equals(ma03 * v100, mb03 * v100));
  EXPECT_TRUE(equals(ma00 * v010, mb00 * v010));
  EXPECT_TRUE(equals(ma01 * v010, mb01 * v010));
  EXPECT_TRUE(equals(ma02 * v010, mb02 * v010));
  EXPECT_TRUE(equals(ma03 * v010, mb03 * v010));
  EXPECT_TRUE(equals(ma00 * v001, mb00 * v001));
  EXPECT_TRUE(equals(ma01 * v001, mb01 * v001));
  EXPECT_TRUE(equals(ma02 * v001, mb02 * v001));
  EXPECT_TRUE(equals(ma03 * v001, mb03 * v001));
  EXPECT_TRUE(equals(ma00 * va, mb00 * va));
  EXPECT_TRUE(equals(ma01 * va, mb01 * va));
  EXPECT_TRUE(equals(ma02 * va, mb02 * va));
  EXPECT_TRUE(equals(ma03 * va, mb03 * va));
  EXPECT_TRUE(equals(ma00 * vb, mb00 * vb));
  EXPECT_TRUE(equals(ma01 * vb, mb01 * vb));
  EXPECT_TRUE(equals(ma02 * vb, mb02 * vb));
  EXPECT_TRUE(equals(ma03 * vb, mb03 * vb));
  EXPECT_TRUE(equals(ma00 * vc, mb00 * vc));
  EXPECT_TRUE(equals(ma01 * vc, mb01 * vc));
  EXPECT_TRUE(equals(ma02 * vc, mb02 * vc));
  EXPECT_TRUE(equals(ma03 * vc, mb03 * vc));
}

TEST(EulerAngleToRotationMatrix, XZY) {
  auto e00 = EulerAngle(0, 0, 0, EulerOrder::XZY);
  auto ma00 = toRotationMatrix(e00);
  auto mb00 = calculateRotationMatrix(e00);
  auto e01 = EulerAngle(PI * 0.333, PI * 0.777, PI * 1.222, EulerOrder::XZY);
  auto ma01 = toRotationMatrix(e01);
  auto mb01 = calculateRotationMatrix(e01);
  auto e02 = EulerAngle(PI * 0.777, PI * 1.222, PI * 0.333, EulerOrder::XZY);
  auto ma02 = toRotationMatrix(e02);
  auto mb02 = calculateRotationMatrix(e02);
  auto e03 = EulerAngle(PI * 1.222, PI * 0.333, PI * 0.777, EulerOrder::XZY);
  auto ma03 = toRotationMatrix(e03);
  auto mb03 = calculateRotationMatrix(e03);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(ma00 * v000, mb00 * v000));
  EXPECT_TRUE(equals(ma01 * v000, mb01 * v000));
  EXPECT_TRUE(equals(ma02 * v000, mb02 * v000));
  EXPECT_TRUE(equals(ma03 * v000, mb03 * v000));
  EXPECT_TRUE(equals(ma00 * v100, mb00 * v100));
  EXPECT_TRUE(equals(ma01 * v100, mb01 * v100));
  EXPECT_TRUE(equals(ma02 * v100, mb02 * v100));
  EXPECT_TRUE(equals(ma03 * v100, mb03 * v100));
  EXPECT_TRUE(equals(ma00 * v010, mb00 * v010));
  EXPECT_TRUE(equals(ma01 * v010, mb01 * v010));
  EXPECT_TRUE(equals(ma02 * v010, mb02 * v010));
  EXPECT_TRUE(equals(ma03 * v010, mb03 * v010));
  EXPECT_TRUE(equals(ma00 * v001, mb00 * v001));
  EXPECT_TRUE(equals(ma01 * v001, mb01 * v001));
  EXPECT_TRUE(equals(ma02 * v001, mb02 * v001));
  EXPECT_TRUE(equals(ma03 * v001, mb03 * v001));
  EXPECT_TRUE(equals(ma00 * va, mb00 * va));
  EXPECT_TRUE(equals(ma01 * va, mb01 * va));
  EXPECT_TRUE(equals(ma02 * va, mb02 * va));
  EXPECT_TRUE(equals(ma03 * va, mb03 * va));
  EXPECT_TRUE(equals(ma00 * vb, mb00 * vb));
  EXPECT_TRUE(equals(ma01 * vb, mb01 * vb));
  EXPECT_TRUE(equals(ma02 * vb, mb02 * vb));
  EXPECT_TRUE(equals(ma03 * vb, mb03 * vb));
  EXPECT_TRUE(equals(ma00 * vc, mb00 * vc));
  EXPECT_TRUE(equals(ma01 * vc, mb01 * vc));
  EXPECT_TRUE(equals(ma02 * vc, mb02 * vc));
  EXPECT_TRUE(equals(ma03 * vc, mb03 * vc));
}

TEST(EulerAngleToRotationMatrix, YXZ) {
  auto e00 = EulerAngle(0, 0, 0, EulerOrder::YXZ);
  auto ma00 = toRotationMatrix(e00);
  auto mb00 = calculateRotationMatrix(e00);
  auto e01 = EulerAngle(PI * 0.333, PI * 0.777, PI * 1.222, EulerOrder::YXZ);
  auto ma01 = toRotationMatrix(e01);
  auto mb01 = calculateRotationMatrix(e01);
  auto e02 = EulerAngle(PI * 0.777, PI * 1.222, PI * 0.333, EulerOrder::YXZ);
  auto ma02 = toRotationMatrix(e02);
  auto mb02 = calculateRotationMatrix(e02);
  auto e03 = EulerAngle(PI * 1.222, PI * 0.333, PI * 0.777, EulerOrder::YXZ);
  auto ma03 = toRotationMatrix(e03);
  auto mb03 = calculateRotationMatrix(e03);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(ma00 * v000, mb00 * v000));
  EXPECT_TRUE(equals(ma01 * v000, mb01 * v000));
  EXPECT_TRUE(equals(ma02 * v000, mb02 * v000));
  EXPECT_TRUE(equals(ma03 * v000, mb03 * v000));
  EXPECT_TRUE(equals(ma00 * v100, mb00 * v100));
  EXPECT_TRUE(equals(ma01 * v100, mb01 * v100));
  EXPECT_TRUE(equals(ma02 * v100, mb02 * v100));
  EXPECT_TRUE(equals(ma03 * v100, mb03 * v100));
  EXPECT_TRUE(equals(ma00 * v010, mb00 * v010));
  EXPECT_TRUE(equals(ma01 * v010, mb01 * v010));
  EXPECT_TRUE(equals(ma02 * v010, mb02 * v010));
  EXPECT_TRUE(equals(ma03 * v010, mb03 * v010));
  EXPECT_TRUE(equals(ma00 * v001, mb00 * v001));
  EXPECT_TRUE(equals(ma01 * v001, mb01 * v001));
  EXPECT_TRUE(equals(ma02 * v001, mb02 * v001));
  EXPECT_TRUE(equals(ma03 * v001, mb03 * v001));
  EXPECT_TRUE(equals(ma00 * va, mb00 * va));
  EXPECT_TRUE(equals(ma01 * va, mb01 * va));
  EXPECT_TRUE(equals(ma02 * va, mb02 * va));
  EXPECT_TRUE(equals(ma03 * va, mb03 * va));
  EXPECT_TRUE(equals(ma00 * vb, mb00 * vb));
  EXPECT_TRUE(equals(ma01 * vb, mb01 * vb));
  EXPECT_TRUE(equals(ma02 * vb, mb02 * vb));
  EXPECT_TRUE(equals(ma03 * vb, mb03 * vb));
  EXPECT_TRUE(equals(ma00 * vc, mb00 * vc));
  EXPECT_TRUE(equals(ma01 * vc, mb01 * vc));
  EXPECT_TRUE(equals(ma02 * vc, mb02 * vc));
  EXPECT_TRUE(equals(ma03 * vc, mb03 * vc));
}

TEST(EulerAngleToRotationMatrix, YZX) {
  auto e00 = EulerAngle(0, 0, 0, EulerOrder::YZX);
  auto ma00 = toRotationMatrix(e00);
  auto mb00 = calculateRotationMatrix(e00);
  auto e01 = EulerAngle(PI * 0.333, PI * 0.777, PI * 1.222, EulerOrder::YZX);
  auto ma01 = toRotationMatrix(e01);
  auto mb01 = calculateRotationMatrix(e01);
  auto e02 = EulerAngle(PI * 0.777, PI * 1.222, PI * 0.333, EulerOrder::YZX);
  auto ma02 = toRotationMatrix(e02);
  auto mb02 = calculateRotationMatrix(e02);
  auto e03 = EulerAngle(PI * 1.222, PI * 0.333, PI * 0.777, EulerOrder::YZX);
  auto ma03 = toRotationMatrix(e03);
  auto mb03 = calculateRotationMatrix(e03);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(ma00 * v000, mb00 * v000));
  EXPECT_TRUE(equals(ma01 * v000, mb01 * v000));
  EXPECT_TRUE(equals(ma02 * v000, mb02 * v000));
  EXPECT_TRUE(equals(ma03 * v000, mb03 * v000));
  EXPECT_TRUE(equals(ma00 * v100, mb00 * v100));
  EXPECT_TRUE(equals(ma01 * v100, mb01 * v100));
  EXPECT_TRUE(equals(ma02 * v100, mb02 * v100));
  EXPECT_TRUE(equals(ma03 * v100, mb03 * v100));
  EXPECT_TRUE(equals(ma00 * v010, mb00 * v010));
  EXPECT_TRUE(equals(ma01 * v010, mb01 * v010));
  EXPECT_TRUE(equals(ma02 * v010, mb02 * v010));
  EXPECT_TRUE(equals(ma03 * v010, mb03 * v010));
  EXPECT_TRUE(equals(ma00 * v001, mb00 * v001));
  EXPECT_TRUE(equals(ma01 * v001, mb01 * v001));
  EXPECT_TRUE(equals(ma02 * v001, mb02 * v001));
  EXPECT_TRUE(equals(ma03 * v001, mb03 * v001));
  EXPECT_TRUE(equals(ma00 * va, mb00 * va));
  EXPECT_TRUE(equals(ma01 * va, mb01 * va));
  EXPECT_TRUE(equals(ma02 * va, mb02 * va));
  EXPECT_TRUE(equals(ma03 * va, mb03 * va));
  EXPECT_TRUE(equals(ma00 * vb, mb00 * vb));
  EXPECT_TRUE(equals(ma01 * vb, mb01 * vb));
  EXPECT_TRUE(equals(ma02 * vb, mb02 * vb));
  EXPECT_TRUE(equals(ma03 * vb, mb03 * vb));
  EXPECT_TRUE(equals(ma00 * vc, mb00 * vc));
  EXPECT_TRUE(equals(ma01 * vc, mb01 * vc));
  EXPECT_TRUE(equals(ma02 * vc, mb02 * vc));
  EXPECT_TRUE(equals(ma03 * vc, mb03 * vc));
}

TEST(EulerAngleToRotationMatrix, ZXY) {
  auto e00 = EulerAngle(0, 0, 0, EulerOrder::ZXY);
  auto ma00 = toRotationMatrix(e00);
  auto mb00 = calculateRotationMatrix(e00);
  auto e01 = EulerAngle(PI * 0.333, PI * 0.777, PI * 1.222, EulerOrder::ZXY);
  auto ma01 = toRotationMatrix(e01);
  auto mb01 = calculateRotationMatrix(e01);
  auto e02 = EulerAngle(PI * 0.777, PI * 1.222, PI * 0.333, EulerOrder::ZXY);
  auto ma02 = toRotationMatrix(e02);
  auto mb02 = calculateRotationMatrix(e02);
  auto e03 = EulerAngle(PI * 1.222, PI * 0.333, PI * 0.777, EulerOrder::ZXY);
  auto ma03 = toRotationMatrix(e03);
  auto mb03 = calculateRotationMatrix(e03);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(ma00 * v000, mb00 * v000));
  EXPECT_TRUE(equals(ma01 * v000, mb01 * v000));
  EXPECT_TRUE(equals(ma02 * v000, mb02 * v000));
  EXPECT_TRUE(equals(ma03 * v000, mb03 * v000));
  EXPECT_TRUE(equals(ma00 * v100, mb00 * v100));
  EXPECT_TRUE(equals(ma01 * v100, mb01 * v100));
  EXPECT_TRUE(equals(ma02 * v100, mb02 * v100));
  EXPECT_TRUE(equals(ma03 * v100, mb03 * v100));
  EXPECT_TRUE(equals(ma00 * v010, mb00 * v010));
  EXPECT_TRUE(equals(ma01 * v010, mb01 * v010));
  EXPECT_TRUE(equals(ma02 * v010, mb02 * v010));
  EXPECT_TRUE(equals(ma03 * v010, mb03 * v010));
  EXPECT_TRUE(equals(ma00 * v001, mb00 * v001));
  EXPECT_TRUE(equals(ma01 * v001, mb01 * v001));
  EXPECT_TRUE(equals(ma02 * v001, mb02 * v001));
  EXPECT_TRUE(equals(ma03 * v001, mb03 * v001));
  EXPECT_TRUE(equals(ma00 * va, mb00 * va));
  EXPECT_TRUE(equals(ma01 * va, mb01 * va));
  EXPECT_TRUE(equals(ma02 * va, mb02 * va));
  EXPECT_TRUE(equals(ma03 * va, mb03 * va));
  EXPECT_TRUE(equals(ma00 * vb, mb00 * vb));
  EXPECT_TRUE(equals(ma01 * vb, mb01 * vb));
  EXPECT_TRUE(equals(ma02 * vb, mb02 * vb));
  EXPECT_TRUE(equals(ma03 * vb, mb03 * vb));
  EXPECT_TRUE(equals(ma00 * vc, mb00 * vc));
  EXPECT_TRUE(equals(ma01 * vc, mb01 * vc));
  EXPECT_TRUE(equals(ma02 * vc, mb02 * vc));
  EXPECT_TRUE(equals(ma03 * vc, mb03 * vc));
}

TEST(EulerAngleToRotationMatrix, ZYX) {
  auto e00 = EulerAngle(0, 0, 0, EulerOrder::ZYX);
  auto ma00 = toRotationMatrix(e00);
  auto mb00 = calculateRotationMatrix(e00);
  auto e01 = EulerAngle(PI * 0.333, PI * 0.777, PI * 1.222, EulerOrder::ZYX);
  auto ma01 = toRotationMatrix(e01);
  auto mb01 = calculateRotationMatrix(e01);
  auto e02 = EulerAngle(PI * 0.777, PI * 1.222, PI * 0.333, EulerOrder::ZYX);
  auto ma02 = toRotationMatrix(e02);
  auto mb02 = calculateRotationMatrix(e02);
  auto e03 = EulerAngle(PI * 1.222, PI * 0.333, PI * 0.777, EulerOrder::ZYX);
  auto ma03 = toRotationMatrix(e03);
  auto mb03 = calculateRotationMatrix(e03);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(ma00 * v000, mb00 * v000));
  EXPECT_TRUE(equals(ma01 * v000, mb01 * v000));
  EXPECT_TRUE(equals(ma02 * v000, mb02 * v000));
  EXPECT_TRUE(equals(ma03 * v000, mb03 * v000));
  EXPECT_TRUE(equals(ma00 * v100, mb00 * v100));
  EXPECT_TRUE(equals(ma01 * v100, mb01 * v100));
  EXPECT_TRUE(equals(ma02 * v100, mb02 * v100));
  EXPECT_TRUE(equals(ma03 * v100, mb03 * v100));
  EXPECT_TRUE(equals(ma00 * v010, mb00 * v010));
  EXPECT_TRUE(equals(ma01 * v010, mb01 * v010));
  EXPECT_TRUE(equals(ma02 * v010, mb02 * v010));
  EXPECT_TRUE(equals(ma03 * v010, mb03 * v010));
  EXPECT_TRUE(equals(ma00 * v001, mb00 * v001));
  EXPECT_TRUE(equals(ma01 * v001, mb01 * v001));
  EXPECT_TRUE(equals(ma02 * v001, mb02 * v001));
  EXPECT_TRUE(equals(ma03 * v001, mb03 * v001));
  EXPECT_TRUE(equals(ma00 * va, mb00 * va));
  EXPECT_TRUE(equals(ma01 * va, mb01 * va));
  EXPECT_TRUE(equals(ma02 * va, mb02 * va));
  EXPECT_TRUE(equals(ma03 * va, mb03 * va));
  EXPECT_TRUE(equals(ma00 * vb, mb00 * vb));
  EXPECT_TRUE(equals(ma01 * vb, mb01 * vb));
  EXPECT_TRUE(equals(ma02 * vb, mb02 * vb));
  EXPECT_TRUE(equals(ma03 * vb, mb03 * vb));
  EXPECT_TRUE(equals(ma00 * vc, mb00 * vc));
  EXPECT_TRUE(equals(ma01 * vc, mb01 * vc));
  EXPECT_TRUE(equals(ma02 * vc, mb02 * vc));
  EXPECT_TRUE(equals(ma03 * vc, mb03 * vc));
}

TEST(EulerAngleToQuaternion, XYZ) {
  auto e00 = EulerAngle(0, 0, 0, EulerOrder::XYZ);
  auto q00 = toQuaternion(e00);
  auto m00 = calculateRotationMatrix(e00);
  auto e01 = EulerAngle(PI * 0.333, PI * 0.777, PI * 1.222, EulerOrder::XYZ);
  auto q01 = toQuaternion(e01);
  auto m01 = calculateRotationMatrix(e01);
  auto e02 = EulerAngle(PI * 0.777, PI * 1.222, PI * 0.333, EulerOrder::XYZ);
  auto q02 = toQuaternion(e02);
  auto m02 = calculateRotationMatrix(e02);
  auto e03 = EulerAngle(PI * 1.222, PI * 0.333, PI * 0.777, EulerOrder::XYZ);
  auto q03 = toQuaternion(e03);
  auto m03 = calculateRotationMatrix(e03);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(q00.rotate(v000), m00 * v000));
  EXPECT_TRUE(equals(q01.rotate(v000), m01 * v000));
  EXPECT_TRUE(equals(q02.rotate(v000), m02 * v000));
  EXPECT_TRUE(equals(q03.rotate(v000), m03 * v000));
  EXPECT_TRUE(equals(q00.rotate(v100), m00 * v100));
  EXPECT_TRUE(equals(q01.rotate(v100), m01 * v100));
  EXPECT_TRUE(equals(q02.rotate(v100), m02 * v100));
  EXPECT_TRUE(equals(q03.rotate(v100), m03 * v100));
  EXPECT_TRUE(equals(q00.rotate(v010), m00 * v010));
  EXPECT_TRUE(equals(q01.rotate(v010), m01 * v010));
  EXPECT_TRUE(equals(q02.rotate(v010), m02 * v010));
  EXPECT_TRUE(equals(q03.rotate(v010), m03 * v010));
  EXPECT_TRUE(equals(q00.rotate(v001), m00 * v001));
  EXPECT_TRUE(equals(q01.rotate(v001), m01 * v001));
  EXPECT_TRUE(equals(q02.rotate(v001), m02 * v001));
  EXPECT_TRUE(equals(q03.rotate(v001), m03 * v001));
  EXPECT_TRUE(equals(q00.rotate(va), m00 * va));
  EXPECT_TRUE(equals(q01.rotate(va), m01 * va));
  EXPECT_TRUE(equals(q02.rotate(va), m02 * va));
  EXPECT_TRUE(equals(q03.rotate(va), m03 * va));
  EXPECT_TRUE(equals(q00.rotate(vb), m00 * vb));
  EXPECT_TRUE(equals(q01.rotate(vb), m01 * vb));
  EXPECT_TRUE(equals(q02.rotate(vb), m02 * vb));
  EXPECT_TRUE(equals(q03.rotate(vb), m03 * vb));
  EXPECT_TRUE(equals(q00.rotate(vc), m00 * vc));
  EXPECT_TRUE(equals(q01.rotate(vc), m01 * vc));
  EXPECT_TRUE(equals(q02.rotate(vc), m02 * vc));
  EXPECT_TRUE(equals(q03.rotate(vc), m03 * vc));
}

TEST(EulerAngleToQuaternion, XZY) {
  auto e00 = EulerAngle(0, 0, 0, EulerOrder::XZY);
  auto q00 = toQuaternion(e00);
  auto m00 = calculateRotationMatrix(e00);
  auto e01 = EulerAngle(PI * 0.333, PI * 0.777, PI * 1.222, EulerOrder::XZY);
  auto q01 = toQuaternion(e01);
  auto m01 = calculateRotationMatrix(e01);
  auto e02 = EulerAngle(PI * 0.777, PI * 1.222, PI * 0.333, EulerOrder::XZY);
  auto q02 = toQuaternion(e02);
  auto m02 = calculateRotationMatrix(e02);
  auto e03 = EulerAngle(PI * 1.222, PI * 0.333, PI * 0.777, EulerOrder::XZY);
  auto q03 = toQuaternion(e03);
  auto m03 = calculateRotationMatrix(e03);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(q00.rotate(v000), m00 * v000));
  EXPECT_TRUE(equals(q01.rotate(v000), m01 * v000));
  EXPECT_TRUE(equals(q02.rotate(v000), m02 * v000));
  EXPECT_TRUE(equals(q03.rotate(v000), m03 * v000));
  EXPECT_TRUE(equals(q00.rotate(v100), m00 * v100));
  EXPECT_TRUE(equals(q01.rotate(v100), m01 * v100));
  EXPECT_TRUE(equals(q02.rotate(v100), m02 * v100));
  EXPECT_TRUE(equals(q03.rotate(v100), m03 * v100));
  EXPECT_TRUE(equals(q00.rotate(v010), m00 * v010));
  EXPECT_TRUE(equals(q01.rotate(v010), m01 * v010));
  EXPECT_TRUE(equals(q02.rotate(v010), m02 * v010));
  EXPECT_TRUE(equals(q03.rotate(v010), m03 * v010));
  EXPECT_TRUE(equals(q00.rotate(v001), m00 * v001));
  EXPECT_TRUE(equals(q01.rotate(v001), m01 * v001));
  EXPECT_TRUE(equals(q02.rotate(v001), m02 * v001));
  EXPECT_TRUE(equals(q03.rotate(v001), m03 * v001));
  EXPECT_TRUE(equals(q00.rotate(va), m00 * va));
  EXPECT_TRUE(equals(q01.rotate(va), m01 * va));
  EXPECT_TRUE(equals(q02.rotate(va), m02 * va));
  EXPECT_TRUE(equals(q03.rotate(va), m03 * va));
  EXPECT_TRUE(equals(q00.rotate(vb), m00 * vb));
  EXPECT_TRUE(equals(q01.rotate(vb), m01 * vb));
  EXPECT_TRUE(equals(q02.rotate(vb), m02 * vb));
  EXPECT_TRUE(equals(q03.rotate(vb), m03 * vb));
  EXPECT_TRUE(equals(q00.rotate(vc), m00 * vc));
  EXPECT_TRUE(equals(q01.rotate(vc), m01 * vc));
  EXPECT_TRUE(equals(q02.rotate(vc), m02 * vc));
  EXPECT_TRUE(equals(q03.rotate(vc), m03 * vc));
}

TEST(EulerAngleToQuaternion, YXZ) {
  auto e00 = EulerAngle(0, 0, 0, EulerOrder::YXZ);
  auto q00 = toQuaternion(e00);
  auto m00 = calculateRotationMatrix(e00);
  auto e01 = EulerAngle(PI * 0.333, PI * 0.777, PI * 1.222, EulerOrder::YXZ);
  auto q01 = toQuaternion(e01);
  auto m01 = calculateRotationMatrix(e01);
  auto e02 = EulerAngle(PI * 0.777, PI * 1.222, PI * 0.333, EulerOrder::YXZ);
  auto q02 = toQuaternion(e02);
  auto m02 = calculateRotationMatrix(e02);
  auto e03 = EulerAngle(PI * 1.222, PI * 0.333, PI * 0.777, EulerOrder::YXZ);
  auto q03 = toQuaternion(e03);
  auto m03 = calculateRotationMatrix(e03);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(q00.rotate(v000), m00 * v000));
  EXPECT_TRUE(equals(q01.rotate(v000), m01 * v000));
  EXPECT_TRUE(equals(q02.rotate(v000), m02 * v000));
  EXPECT_TRUE(equals(q03.rotate(v000), m03 * v000));
  EXPECT_TRUE(equals(q00.rotate(v100), m00 * v100));
  EXPECT_TRUE(equals(q01.rotate(v100), m01 * v100));
  EXPECT_TRUE(equals(q02.rotate(v100), m02 * v100));
  EXPECT_TRUE(equals(q03.rotate(v100), m03 * v100));
  EXPECT_TRUE(equals(q00.rotate(v010), m00 * v010));
  EXPECT_TRUE(equals(q01.rotate(v010), m01 * v010));
  EXPECT_TRUE(equals(q02.rotate(v010), m02 * v010));
  EXPECT_TRUE(equals(q03.rotate(v010), m03 * v010));
  EXPECT_TRUE(equals(q00.rotate(v001), m00 * v001));
  EXPECT_TRUE(equals(q01.rotate(v001), m01 * v001));
  EXPECT_TRUE(equals(q02.rotate(v001), m02 * v001));
  EXPECT_TRUE(equals(q03.rotate(v001), m03 * v001));
  EXPECT_TRUE(equals(q00.rotate(va), m00 * va));
  EXPECT_TRUE(equals(q01.rotate(va), m01 * va));
  EXPECT_TRUE(equals(q02.rotate(va), m02 * va));
  EXPECT_TRUE(equals(q03.rotate(va), m03 * va));
  EXPECT_TRUE(equals(q00.rotate(vb), m00 * vb));
  EXPECT_TRUE(equals(q01.rotate(vb), m01 * vb));
  EXPECT_TRUE(equals(q02.rotate(vb), m02 * vb));
  EXPECT_TRUE(equals(q03.rotate(vb), m03 * vb));
  EXPECT_TRUE(equals(q00.rotate(vc), m00 * vc));
  EXPECT_TRUE(equals(q01.rotate(vc), m01 * vc));
  EXPECT_TRUE(equals(q02.rotate(vc), m02 * vc));
  EXPECT_TRUE(equals(q03.rotate(vc), m03 * vc));
}

TEST(EulerAngleToQuaternion, YZX) {
  auto e00 = EulerAngle(0, 0, 0, EulerOrder::YZX);
  auto q00 = toQuaternion(e00);
  auto m00 = calculateRotationMatrix(e00);
  auto e01 = EulerAngle(PI * 0.333, PI * 0.777, PI * 1.222, EulerOrder::YZX);
  auto q01 = toQuaternion(e01);
  auto m01 = calculateRotationMatrix(e01);
  auto e02 = EulerAngle(PI * 0.777, PI * 1.222, PI * 0.333, EulerOrder::YZX);
  auto q02 = toQuaternion(e02);
  auto m02 = calculateRotationMatrix(e02);
  auto e03 = EulerAngle(PI * 1.222, PI * 0.333, PI * 0.777, EulerOrder::YZX);
  auto q03 = toQuaternion(e03);
  auto m03 = calculateRotationMatrix(e03);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(q00.rotate(v000), m00 * v000));
  EXPECT_TRUE(equals(q01.rotate(v000), m01 * v000));
  EXPECT_TRUE(equals(q02.rotate(v000), m02 * v000));
  EXPECT_TRUE(equals(q03.rotate(v000), m03 * v000));
  EXPECT_TRUE(equals(q00.rotate(v100), m00 * v100));
  EXPECT_TRUE(equals(q01.rotate(v100), m01 * v100));
  EXPECT_TRUE(equals(q02.rotate(v100), m02 * v100));
  EXPECT_TRUE(equals(q03.rotate(v100), m03 * v100));
  EXPECT_TRUE(equals(q00.rotate(v010), m00 * v010));
  EXPECT_TRUE(equals(q01.rotate(v010), m01 * v010));
  EXPECT_TRUE(equals(q02.rotate(v010), m02 * v010));
  EXPECT_TRUE(equals(q03.rotate(v010), m03 * v010));
  EXPECT_TRUE(equals(q00.rotate(v001), m00 * v001));
  EXPECT_TRUE(equals(q01.rotate(v001), m01 * v001));
  EXPECT_TRUE(equals(q02.rotate(v001), m02 * v001));
  EXPECT_TRUE(equals(q03.rotate(v001), m03 * v001));
  EXPECT_TRUE(equals(q00.rotate(va), m00 * va));
  EXPECT_TRUE(equals(q01.rotate(va), m01 * va));
  EXPECT_TRUE(equals(q02.rotate(va), m02 * va));
  EXPECT_TRUE(equals(q03.rotate(va), m03 * va));
  EXPECT_TRUE(equals(q00.rotate(vb), m00 * vb));
  EXPECT_TRUE(equals(q01.rotate(vb), m01 * vb));
  EXPECT_TRUE(equals(q02.rotate(vb), m02 * vb));
  EXPECT_TRUE(equals(q03.rotate(vb), m03 * vb));
  EXPECT_TRUE(equals(q00.rotate(vc), m00 * vc));
  EXPECT_TRUE(equals(q01.rotate(vc), m01 * vc));
  EXPECT_TRUE(equals(q02.rotate(vc), m02 * vc));
  EXPECT_TRUE(equals(q03.rotate(vc), m03 * vc));
}

TEST(EulerAngleToQuaternion, ZXY) {
  auto e00 = EulerAngle(0, 0, 0, EulerOrder::ZXY);
  auto q00 = toQuaternion(e00);
  auto m00 = calculateRotationMatrix(e00);
  auto e01 = EulerAngle(PI * 0.333, PI * 0.777, PI * 1.222, EulerOrder::ZXY);
  auto q01 = toQuaternion(e01);
  auto m01 = calculateRotationMatrix(e01);
  auto e02 = EulerAngle(PI * 0.777, PI * 1.222, PI * 0.333, EulerOrder::ZXY);
  auto q02 = toQuaternion(e02);
  auto m02 = calculateRotationMatrix(e02);
  auto e03 = EulerAngle(PI * 1.222, PI * 0.333, PI * 0.777, EulerOrder::ZXY);
  auto q03 = toQuaternion(e03);
  auto m03 = calculateRotationMatrix(e03);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(q00.rotate(v000), m00 * v000));
  EXPECT_TRUE(equals(q01.rotate(v000), m01 * v000));
  EXPECT_TRUE(equals(q02.rotate(v000), m02 * v000));
  EXPECT_TRUE(equals(q03.rotate(v000), m03 * v000));
  EXPECT_TRUE(equals(q00.rotate(v100), m00 * v100));
  EXPECT_TRUE(equals(q01.rotate(v100), m01 * v100));
  EXPECT_TRUE(equals(q02.rotate(v100), m02 * v100));
  EXPECT_TRUE(equals(q03.rotate(v100), m03 * v100));
  EXPECT_TRUE(equals(q00.rotate(v010), m00 * v010));
  EXPECT_TRUE(equals(q01.rotate(v010), m01 * v010));
  EXPECT_TRUE(equals(q02.rotate(v010), m02 * v010));
  EXPECT_TRUE(equals(q03.rotate(v010), m03 * v010));
  EXPECT_TRUE(equals(q00.rotate(v001), m00 * v001));
  EXPECT_TRUE(equals(q01.rotate(v001), m01 * v001));
  EXPECT_TRUE(equals(q02.rotate(v001), m02 * v001));
  EXPECT_TRUE(equals(q03.rotate(v001), m03 * v001));
  EXPECT_TRUE(equals(q00.rotate(va), m00 * va));
  EXPECT_TRUE(equals(q01.rotate(va), m01 * va));
  EXPECT_TRUE(equals(q02.rotate(va), m02 * va));
  EXPECT_TRUE(equals(q03.rotate(va), m03 * va));
  EXPECT_TRUE(equals(q00.rotate(vb), m00 * vb));
  EXPECT_TRUE(equals(q01.rotate(vb), m01 * vb));
  EXPECT_TRUE(equals(q02.rotate(vb), m02 * vb));
  EXPECT_TRUE(equals(q03.rotate(vb), m03 * vb));
  EXPECT_TRUE(equals(q00.rotate(vc), m00 * vc));
  EXPECT_TRUE(equals(q01.rotate(vc), m01 * vc));
  EXPECT_TRUE(equals(q02.rotate(vc), m02 * vc));
  EXPECT_TRUE(equals(q03.rotate(vc), m03 * vc));
}

TEST(EulerAngleToQuaternion, ZYX) {
  auto e00 = EulerAngle(0, 0, 0, EulerOrder::ZYX);
  auto q00 = toQuaternion(e00);
  auto m00 = calculateRotationMatrix(e00);
  auto e01 = EulerAngle(PI * 0.333, PI * 0.777, PI * 1.222, EulerOrder::ZYX);
  auto q01 = toQuaternion(e01);
  auto m01 = calculateRotationMatrix(e01);
  auto e02 = EulerAngle(PI * 0.777, PI * 1.222, PI * 0.333, EulerOrder::ZYX);
  auto q02 = toQuaternion(e02);
  auto m02 = calculateRotationMatrix(e02);
  auto e03 = EulerAngle(PI * 1.222, PI * 0.333, PI * 0.777, EulerOrder::ZYX);
  auto q03 = toQuaternion(e03);
  auto m03 = calculateRotationMatrix(e03);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(q00.rotate(v000), m00 * v000));
  EXPECT_TRUE(equals(q01.rotate(v000), m01 * v000));
  EXPECT_TRUE(equals(q02.rotate(v000), m02 * v000));
  EXPECT_TRUE(equals(q03.rotate(v000), m03 * v000));
  EXPECT_TRUE(equals(q00.rotate(v100), m00 * v100));
  EXPECT_TRUE(equals(q01.rotate(v100), m01 * v100));
  EXPECT_TRUE(equals(q02.rotate(v100), m02 * v100));
  EXPECT_TRUE(equals(q03.rotate(v100), m03 * v100));
  EXPECT_TRUE(equals(q00.rotate(v010), m00 * v010));
  EXPECT_TRUE(equals(q01.rotate(v010), m01 * v010));
  EXPECT_TRUE(equals(q02.rotate(v010), m02 * v010));
  EXPECT_TRUE(equals(q03.rotate(v010), m03 * v010));
  EXPECT_TRUE(equals(q00.rotate(v001), m00 * v001));
  EXPECT_TRUE(equals(q01.rotate(v001), m01 * v001));
  EXPECT_TRUE(equals(q02.rotate(v001), m02 * v001));
  EXPECT_TRUE(equals(q03.rotate(v001), m03 * v001));
  EXPECT_TRUE(equals(q00.rotate(va), m00 * va));
  EXPECT_TRUE(equals(q01.rotate(va), m01 * va));
  EXPECT_TRUE(equals(q02.rotate(va), m02 * va));
  EXPECT_TRUE(equals(q03.rotate(va), m03 * va));
  EXPECT_TRUE(equals(q00.rotate(vb), m00 * vb));
  EXPECT_TRUE(equals(q01.rotate(vb), m01 * vb));
  EXPECT_TRUE(equals(q02.rotate(vb), m02 * vb));
  EXPECT_TRUE(equals(q03.rotate(vb), m03 * vb));
  EXPECT_TRUE(equals(q00.rotate(vc), m00 * vc));
  EXPECT_TRUE(equals(q01.rotate(vc), m01 * vc));
  EXPECT_TRUE(equals(q02.rotate(vc), m02 * vc));
  EXPECT_TRUE(equals(q03.rotate(vc), m03 * vc));
}

TEST(QuaternionToRotationMatrix, RotationX) {
  auto qx0 = Quaternion::rotationX(0);
  auto mx0 = toRotationMatrix(qx0);
  auto qx90 = Quaternion::rotationX(HALF_PI);
  auto mx90 = toRotationMatrix(qx90);
  auto qx180 = Quaternion::rotationX(2.0f * HALF_PI);
  auto mx180 = toRotationMatrix(qx180);
  auto qx270 = Quaternion::rotationX(3.0f * HALF_PI);
  auto mx270 = toRotationMatrix(qx270);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(qx0.rotate(v000), mx0 * v000));
  EXPECT_TRUE(equals(qx90.rotate(v000), mx90 * v000));
  EXPECT_TRUE(equals(qx180.rotate(v000), mx180 * v000));
  EXPECT_TRUE(equals(qx270.rotate(v000), mx270 * v000));
  EXPECT_TRUE(equals(qx0.rotate(v100), mx0 * v100));
  EXPECT_TRUE(equals(qx90.rotate(v100), mx90 * v100));
  EXPECT_TRUE(equals(qx180.rotate(v100), mx180 * v100));
  EXPECT_TRUE(equals(qx270.rotate(v100), mx270 * v100));
  EXPECT_TRUE(equals(qx0.rotate(v010), mx0 * v010));
  EXPECT_TRUE(equals(qx90.rotate(v010), mx90 * v010));
  EXPECT_TRUE(equals(qx180.rotate(v010), mx180 * v010));
  EXPECT_TRUE(equals(qx270.rotate(v010), mx270 * v010));
  EXPECT_TRUE(equals(qx0.rotate(v001), mx0 * v001));
  EXPECT_TRUE(equals(qx90.rotate(v001), mx90 * v001));
  EXPECT_TRUE(equals(qx180.rotate(v001), mx180 * v001));
  EXPECT_TRUE(equals(qx270.rotate(v001), mx270 * v001));
  EXPECT_TRUE(equals(qx0.rotate(va), mx0 * va));
  EXPECT_TRUE(equals(qx90.rotate(va), mx90 * va));
  EXPECT_TRUE(equals(qx180.rotate(va), mx180 * va));
  EXPECT_TRUE(equals(qx270.rotate(va), mx270 * va));
  EXPECT_TRUE(equals(qx0.rotate(vb), mx0 * vb));
  EXPECT_TRUE(equals(qx90.rotate(vb), mx90 * vb));
  EXPECT_TRUE(equals(qx180.rotate(vb), mx180 * vb));
  EXPECT_TRUE(equals(qx270.rotate(vb), mx270 * vb));
  EXPECT_TRUE(equals(qx0.rotate(vc), mx0 * vc));
  EXPECT_TRUE(equals(qx90.rotate(vc), mx90 * vc));
  EXPECT_TRUE(equals(qx180.rotate(vc), mx180 * vc));
  EXPECT_TRUE(equals(qx270.rotate(vc), mx270 * vc));
}

TEST(QuaternionToRotationMatrix, RotationY) {
  auto qx0 = Quaternion::rotationY(0);
  auto mx0 = toRotationMatrix(qx0);
  auto qx90 = Quaternion::rotationY(HALF_PI);
  auto mx90 = toRotationMatrix(qx90);
  auto qx180 = Quaternion::rotationY(2.0f * HALF_PI);
  auto mx180 = toRotationMatrix(qx180);
  auto qx270 = Quaternion::rotationY(3.0f * HALF_PI);
  auto mx270 = toRotationMatrix(qx270);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);
  
  EXPECT_TRUE(equals(qx0.rotate(v000), mx0 * v000));
  EXPECT_TRUE(equals(qx90.rotate(v000), mx90 * v000));
  EXPECT_TRUE(equals(qx180.rotate(v000), mx180 * v000));
  EXPECT_TRUE(equals(qx270.rotate(v000), mx270 * v000));
  EXPECT_TRUE(equals(qx0.rotate(v100), mx0 * v100));
  EXPECT_TRUE(equals(qx90.rotate(v100), mx90 * v100));
  EXPECT_TRUE(equals(qx180.rotate(v100), mx180 * v100));
  EXPECT_TRUE(equals(qx270.rotate(v100), mx270 * v100));
  EXPECT_TRUE(equals(qx0.rotate(v010), mx0 * v010));
  EXPECT_TRUE(equals(qx90.rotate(v010), mx90 * v010));
  EXPECT_TRUE(equals(qx180.rotate(v010), mx180 * v010));
  EXPECT_TRUE(equals(qx270.rotate(v010), mx270 * v010));
  EXPECT_TRUE(equals(qx0.rotate(v001), mx0 * v001));
  EXPECT_TRUE(equals(qx90.rotate(v001), mx90 * v001));
  EXPECT_TRUE(equals(qx180.rotate(v001), mx180 * v001));
  EXPECT_TRUE(equals(qx270.rotate(v001), mx270 * v001));
  EXPECT_TRUE(equals(qx0.rotate(va), mx0 * va));
  EXPECT_TRUE(equals(qx90.rotate(va), mx90 * va));
  EXPECT_TRUE(equals(qx180.rotate(va), mx180 * va));
  EXPECT_TRUE(equals(qx270.rotate(va), mx270 * va));
  EXPECT_TRUE(equals(qx0.rotate(vb), mx0 * vb));
  EXPECT_TRUE(equals(qx90.rotate(vb), mx90 * vb));
  EXPECT_TRUE(equals(qx180.rotate(vb), mx180 * vb));
  EXPECT_TRUE(equals(qx270.rotate(vb), mx270 * vb));
  EXPECT_TRUE(equals(qx0.rotate(vc), mx0 * vc));
  EXPECT_TRUE(equals(qx90.rotate(vc), mx90 * vc));
  EXPECT_TRUE(equals(qx180.rotate(vc), mx180 * vc));
  EXPECT_TRUE(equals(qx270.rotate(vc), mx270 * vc));
}

TEST(QuaternionToRotationMatrix, RotationZ) {
  auto qx0 = Quaternion::rotationZ(0);
  auto mx0 = toRotationMatrix(qx0);
  auto qx90 = Quaternion::rotationZ(HALF_PI);
  auto mx90 = toRotationMatrix(qx90);
  auto qx180 = Quaternion::rotationZ(2.0f * HALF_PI);
  auto mx180 = toRotationMatrix(qx180);
  auto qx270 = Quaternion::rotationZ(3.0f * HALF_PI);
  auto mx270 = toRotationMatrix(qx270);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(qx0.rotate(v000), mx0 * v000));
  EXPECT_TRUE(equals(qx90.rotate(v000), mx90 * v000));
  EXPECT_TRUE(equals(qx180.rotate(v000), mx180 * v000));
  EXPECT_TRUE(equals(qx270.rotate(v000), mx270 * v000));
  EXPECT_TRUE(equals(qx0.rotate(v100), mx0 * v100));
  EXPECT_TRUE(equals(qx90.rotate(v100), mx90 * v100));
  EXPECT_TRUE(equals(qx180.rotate(v100), mx180 * v100));
  EXPECT_TRUE(equals(qx270.rotate(v100), mx270 * v100));
  EXPECT_TRUE(equals(qx0.rotate(v010), mx0 * v010));
  EXPECT_TRUE(equals(qx90.rotate(v010), mx90 * v010));
  EXPECT_TRUE(equals(qx180.rotate(v010), mx180 * v010));
  EXPECT_TRUE(equals(qx270.rotate(v010), mx270 * v010));
  EXPECT_TRUE(equals(qx0.rotate(v001), mx0 * v001));
  EXPECT_TRUE(equals(qx90.rotate(v001), mx90 * v001));
  EXPECT_TRUE(equals(qx180.rotate(v001), mx180 * v001));
  EXPECT_TRUE(equals(qx270.rotate(v001), mx270 * v001));
  EXPECT_TRUE(equals(qx0.rotate(va), mx0 * va));
  EXPECT_TRUE(equals(qx90.rotate(va), mx90 * va));
  EXPECT_TRUE(equals(qx180.rotate(va), mx180 * va));
  EXPECT_TRUE(equals(qx270.rotate(va), mx270 * va));
  EXPECT_TRUE(equals(qx0.rotate(vb), mx0 * vb));
  EXPECT_TRUE(equals(qx90.rotate(vb), mx90 * vb));
  EXPECT_TRUE(equals(qx180.rotate(vb), mx180 * vb));
  EXPECT_TRUE(equals(qx270.rotate(vb), mx270 * vb));
  EXPECT_TRUE(equals(qx0.rotate(vc), mx0 * vc));
  EXPECT_TRUE(equals(qx90.rotate(vc), mx90 * vc));
  EXPECT_TRUE(equals(qx180.rotate(vc), mx180 * vc));
  EXPECT_TRUE(equals(qx270.rotate(vc), mx270 * vc));
}

TEST(QuaternionToRotationMatrix, Others) {
  auto q0 = calculateQuaternion(EulerAngle(0.333 * PI, 0.777 * PI, 1.888 * PI, EulerOrder::XYZ));
  auto m0 = toRotationMatrix(q0);
  auto q1 = calculateQuaternion(EulerAngle(0.333 * PI, 0.777 * PI, 1.888 * PI, EulerOrder::XZY));
  auto m1 = toRotationMatrix(q1); 
  auto q2 = calculateQuaternion(EulerAngle(0.333 * PI, 0.777 * PI, 1.888 * PI, EulerOrder::YXZ));
  auto m2 = toRotationMatrix(q2);
  auto q3 = calculateQuaternion(EulerAngle(0.333 * PI, 0.777 * PI, 1.888 * PI, EulerOrder::YZX));
  auto m3 = toRotationMatrix(q3);
  auto q4 = calculateQuaternion(EulerAngle(0.333 * PI, 0.777 * PI, 1.888 * PI, EulerOrder::ZXY));
  auto m4 = toRotationMatrix(q4);
  auto q5 = calculateQuaternion(EulerAngle(0.333 * PI, 0.777 * PI, 1.888 * PI, EulerOrder::ZYX));
  auto m5 = toRotationMatrix(q5);


  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  auto va = Vector3(2, 3, 5);
  auto vb = Vector3(-7, 11, 13);
  auto vc = Vector3(-17, -19, 23);

  EXPECT_TRUE(equals(q0.rotate(v000), m0 * v000));
  EXPECT_TRUE(equals(q1.rotate(v000), m1 * v000));
  EXPECT_TRUE(equals(q2.rotate(v000), m2 * v000));
  EXPECT_TRUE(equals(q3.rotate(v000), m3 * v000));
  EXPECT_TRUE(equals(q4.rotate(v000), m4 * v000));
  EXPECT_TRUE(equals(q5.rotate(v000), m5 * v000));
  EXPECT_TRUE(equals(q0.rotate(v100), m0 * v100));
  EXPECT_TRUE(equals(q1.rotate(v100), m1 * v100));
  EXPECT_TRUE(equals(q2.rotate(v100), m2 * v100));
  EXPECT_TRUE(equals(q3.rotate(v100), m3 * v100));
  EXPECT_TRUE(equals(q4.rotate(v100), m4 * v100));
  EXPECT_TRUE(equals(q5.rotate(v100), m5 * v100));
  EXPECT_TRUE(equals(q0.rotate(v010), m0 * v010));
  EXPECT_TRUE(equals(q1.rotate(v010), m1 * v010));
  EXPECT_TRUE(equals(q2.rotate(v010), m2 * v010));
  EXPECT_TRUE(equals(q3.rotate(v010), m3 * v010));
  EXPECT_TRUE(equals(q4.rotate(v010), m4 * v010));
  EXPECT_TRUE(equals(q5.rotate(v010), m5 * v010));
  EXPECT_TRUE(equals(q0.rotate(v001), m0 * v001));
  EXPECT_TRUE(equals(q1.rotate(v001), m1 * v001));
  EXPECT_TRUE(equals(q2.rotate(v001), m2 * v001));
  EXPECT_TRUE(equals(q3.rotate(v001), m3 * v001));
  EXPECT_TRUE(equals(q4.rotate(v001), m4 * v001));
  EXPECT_TRUE(equals(q5.rotate(v001), m5 * v001));
  EXPECT_TRUE(equals(q0.rotate(va), m0 * va));
  EXPECT_TRUE(equals(q1.rotate(va), m1 * va));
  EXPECT_TRUE(equals(q2.rotate(va), m2 * va));
  EXPECT_TRUE(equals(q3.rotate(va), m3 * va));
  EXPECT_TRUE(equals(q4.rotate(va), m4 * va));
  EXPECT_TRUE(equals(q5.rotate(va), m5 * va));
  EXPECT_TRUE(equals(q0.rotate(vb), m0 * vb));
  EXPECT_TRUE(equals(q1.rotate(vb), m1 * vb));
  EXPECT_TRUE(equals(q2.rotate(vb), m2 * vb));
  EXPECT_TRUE(equals(q3.rotate(vb), m3 * vb));
  EXPECT_TRUE(equals(q4.rotate(vb), m4 * vb));
  EXPECT_TRUE(equals(q5.rotate(vb), m5 * vb));
  EXPECT_TRUE(equals(q0.rotate(vc), m0 * vc));
  EXPECT_TRUE(equals(q1.rotate(vc), m1 * vc));
  EXPECT_TRUE(equals(q2.rotate(vc), m2 * vc));
  EXPECT_TRUE(equals(q3.rotate(vc), m3 * vc));
  EXPECT_TRUE(equals(q4.rotate(vc), m4 * vc));
  EXPECT_TRUE(equals(q5.rotate(vc), m5 * vc));
}

TEST(RotationMatrixToQuaternion, RotationX) {
  auto mx0 = RotationMatrix::rotationX(0);
  auto qx0 = toQuaternion(mx0);
  auto mx90 = RotationMatrix::rotationX(HALF_PI);
  auto qx90 = toQuaternion(mx90);
  auto mx180 = RotationMatrix::rotationX(2.0f * HALF_PI);
  auto qx180 = toQuaternion(mx180);
  auto mx270 = RotationMatrix::rotationX(3.0f * HALF_PI);
  auto qx270 = toQuaternion(mx270);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  EXPECT_TRUE(equals(mx0 * v000, qx0.rotate(v000)));
  EXPECT_TRUE(equals(mx90 * v000, qx90.rotate(v000)));
  EXPECT_TRUE(equals(mx180 * v000, qx180.rotate(v000)));
  EXPECT_TRUE(equals(mx270 * v000, qx270.rotate(v000)));
  EXPECT_TRUE(equals(mx0 * v100, qx0.rotate(v100)));
  EXPECT_TRUE(equals(mx90 * v100, qx90.rotate(v100)));
  EXPECT_TRUE(equals(mx180 * v100, qx180.rotate(v100)));
  EXPECT_TRUE(equals(mx270 * v100, qx270.rotate(v100)));
  EXPECT_TRUE(equals(mx0 * v010, qx0.rotate(v010)));
  EXPECT_TRUE(equals(mx90 * v010, qx90.rotate(v010)));
  EXPECT_TRUE(equals(mx180 * v010, qx180.rotate(v010)));
  EXPECT_TRUE(equals(mx270 * v010, qx270.rotate(v010)));
  EXPECT_TRUE(equals(mx0 * v001, qx0.rotate(v001)));
  EXPECT_TRUE(equals(mx90 * v001, qx90.rotate(v001)));
  EXPECT_TRUE(equals(mx180 * v001, qx180.rotate(v001)));
  EXPECT_TRUE(equals(mx270 * v001, qx270.rotate(v001)));
}

TEST(RotationMatrixToQuaternion, RotationY) {
  auto mx0 = RotationMatrix::rotationY(0);
  auto qx0 = toQuaternion(mx0);
  auto mx90 = RotationMatrix::rotationY(HALF_PI);
  auto qx90 = toQuaternion(mx90);
  auto mx180 = RotationMatrix::rotationY(2.0f * HALF_PI);
  auto qx180 = toQuaternion(mx180);
  auto mx270 = RotationMatrix::rotationY(3.0f * HALF_PI);
  auto qx270 = toQuaternion(mx270);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  EXPECT_TRUE(equals(mx0 * v000, qx0.rotate(v000)));
  EXPECT_TRUE(equals(mx90 * v000, qx90.rotate(v000)));
  EXPECT_TRUE(equals(mx180 * v000, qx180.rotate(v000)));
  EXPECT_TRUE(equals(mx270 * v000, qx270.rotate(v000)));
  EXPECT_TRUE(equals(mx0 * v100, qx0.rotate(v100)));
  EXPECT_TRUE(equals(mx90 * v100, qx90.rotate(v100)));
  EXPECT_TRUE(equals(mx180 * v100, qx180.rotate(v100)));
  EXPECT_TRUE(equals(mx270 * v100, qx270.rotate(v100)));
  EXPECT_TRUE(equals(mx0 * v010, qx0.rotate(v010)));
  EXPECT_TRUE(equals(mx90 * v010, qx90.rotate(v010)));
  EXPECT_TRUE(equals(mx180 * v010, qx180.rotate(v010)));
  EXPECT_TRUE(equals(mx270 * v010, qx270.rotate(v010)));
  EXPECT_TRUE(equals(mx0 * v001, qx0.rotate(v001)));
  EXPECT_TRUE(equals(mx90 * v001, qx90.rotate(v001)));
  EXPECT_TRUE(equals(mx180 * v001, qx180.rotate(v001)));
  EXPECT_TRUE(equals(mx270 * v001, qx270.rotate(v001)));
}

TEST(RotationMatrixToQuaternion, RotationZ) {
  auto mx0 = RotationMatrix::rotationZ(0);
  auto qx0 = toQuaternion(mx0);
  auto mx90 = RotationMatrix::rotationZ(HALF_PI);
  auto qx90 = toQuaternion(mx90);
  auto mx180 = RotationMatrix::rotationZ(2.0f * HALF_PI);
  auto qx180 = toQuaternion(mx180);
  auto mx270 = RotationMatrix::rotationZ(3.0f * HALF_PI);
  auto qx270 = toQuaternion(mx270);

  auto v000 = Vector3(0, 0, 0);
  auto v100 = Vector3(1, 0, 0);
  auto v010 = Vector3(0, 1, 0);
  auto v001 = Vector3(0, 0, 1);
  EXPECT_TRUE(equals(mx0 * v000, qx0.rotate(v000)));
  EXPECT_TRUE(equals(mx90 * v000, qx90.rotate(v000)));
  EXPECT_TRUE(equals(mx180 * v000, qx180.rotate(v000)));
  EXPECT_TRUE(equals(mx270 * v000, qx270.rotate(v000)));
  EXPECT_TRUE(equals(mx0 * v100, qx0.rotate(v100)));
  EXPECT_TRUE(equals(mx90 * v100, qx90.rotate(v100)));
  EXPECT_TRUE(equals(mx180 * v100, qx180.rotate(v100)));
  EXPECT_TRUE(equals(mx270 * v100, qx270.rotate(v100)));
  EXPECT_TRUE(equals(mx0 * v010, qx0.rotate(v010)));
  EXPECT_TRUE(equals(mx90 * v010, qx90.rotate(v010)));
  EXPECT_TRUE(equals(mx180 * v010, qx180.rotate(v010)));
  EXPECT_TRUE(equals(mx270 * v010, qx270.rotate(v010)));
  EXPECT_TRUE(equals(mx0 * v001, qx0.rotate(v001)));
  EXPECT_TRUE(equals(mx90 * v001, qx90.rotate(v001)));
  EXPECT_TRUE(equals(mx180 * v001, qx180.rotate(v001)));
  EXPECT_TRUE(equals(mx270 * v001, qx270.rotate(v001)));
}