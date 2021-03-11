#ifndef __EULERANGLE_H__
#define __EULERANGLE_H__

enum class EulerOrder {
  XYZ,
  XZY,
  YXZ,
  YZX,
  ZXY,
  ZYX
};

class EulerAngle {
public:
  float x;
  float y;
  float z;
  EulerOrder order;
  EulerAngle(float x, float y, float z, EulerOrder order): x(x), y(y), z(z), order(order) {}
};

#endif // __EULERANGLE_H__