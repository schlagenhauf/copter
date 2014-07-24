/*
 * A PID controller functor
 */

#include "Vec3.h"

class Pid
{
 public:
  Pid(float coeffP, coeffI, coeffD) : coeffP_(coeffP), coeffI_(coeffI), coeffD_(coeffD) {}
  Vec3 operator()(const Quaternion& actual, const Quaternion& target);

 private:
  float coeffP_, coeffI_, coeffD_;

  Quaternion integError, lastError;

};

Vec3 Pid::operator()(const Quaternion& actual, const Quaternion& target)
{
  Quaternion error = target * actual.inverse();
  Quaternion dError = error * lastError.inverse();
  integError = integError * error;

  Quaternion delta = (error * coeffP_) * (integError * coeffI_) * (dError * coeffD_);

  Vec3 rpy;
  rpy.x = atan2(2 * delta.y*delta.w - 2 * delta.x*delta.z, 1 - 2 * delta.y * delta.y - 2 * delta.z * delta.z);
  rpy.y = asin(2 * delta.x*delta.y + 2 * delta.z*delta.w);
  rpy.z = atan2(2 * delta.x*delta.w - 2 * delta.y*delta.z, 1 - 2 * delta.x * delta.x - 2 * delta.z * delta.z);

  return rpy;
}
