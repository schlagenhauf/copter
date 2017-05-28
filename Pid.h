/*
 * A PID controller functor
 */

#ifdef _GTEST_ON
#include <gtest/gtest.h>
#endif

#include "Vec3.h"
#include "Quaternion.h"

using namespace Geometry;

class Pid
{
 public:
#ifdef _GTEST_ON
  FRIEND_TEST(pid_tests, pid_ctor);
#endif
  Pid(double coeffP, double coeffI, double coeffD) : coeffP_(coeffP), coeffI_(coeffI), coeffD_(coeffD)
  {
    integError = 0;
    lastError = 0;
  }
#ifdef _GTEST_ON
  FRIEND_TEST(pid_tests, pid_functor);
#endif
  double operator()(const double& actual, const double& target);

 private:
  double coeffP_, coeffI_, coeffD_;

  double integError, lastError;
};

double Pid::operator()(const double& actual, const double& target)
{
  double error = target - actual;
  double dError = error - lastError;
  integError += error;
  if (integError > 0.05)
    integError = 0.05;

  lastError = error;

  return ((error * coeffP_) + (integError * coeffI_) + (dError * coeffD_));
}
