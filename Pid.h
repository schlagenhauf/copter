/*
 * A PID controller functor
 */

#include <gtest/gtest.h>

#include "Vec3.h"
#include "Quaternion.h"

using namespace Geometry;

class Pid
{
 public:
  FRIEND_TEST(pid_tests, pid_ctor);
  Pid(float coeffP, float coeffI, float coeffD) : coeffP_(coeffP), coeffI_(coeffI), coeffD_(coeffD) {}
  FRIEND_TEST(pid_tests, pid_functor);
  Vec3<float> operator()(const Quaternion& actual, const Quaternion& target);

 private:
  float coeffP_, coeffI_, coeffD_;

  Quaternion integError, lastError;

};

Vec3<float> Pid::operator()(const Quaternion& actual, const Quaternion& target)
{
  Quaternion error = target * actual.inverse();
  Quaternion dError = error * lastError.inverse();
  integError = integError * error;
  lastError = error;

  Quaternion delta = (error * coeffP_) * (integError * coeffI_) * (dError * coeffD_);
  //printf("delta: %f, %f, %f, %f\n", delta.w, delta.x, delta.y, delta.z);

  Vec3<float> rpy;
  rpy.x = atan2(2 * delta.y*delta.w - 2 * delta.x*delta.z, 1 - 2 * delta.y * delta.y - 2 * delta.z * delta.z);
  rpy.y = asin(2 * delta.x*delta.y + 2 * delta.z*delta.w);
  rpy.z = atan2(2 * delta.x*delta.w - 2 * delta.y*delta.z, 1 - 2 * delta.x * delta.x - 2 * delta.z * delta.z);

  //printf("rpy: %f, %f, %f\n", rpy.x, rpy.y, rpy.z);

  return rpy;
}
