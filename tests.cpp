/*
 * tests.cpp
 * Unit tests for Pid.h, ...
 */

#include <gtest/gtest.h>
#define USE_MATH_DEFINES_
#include <math.h>

#define _GTEST_ON

#include "Quaternion.h"
#include "Pid.h"

using namespace Geometry;

TEST(quaternion_tests, quaternion_ctors)
{
  Quaternion qt1;
  ASSERT_FLOAT_EQ(qt1.w, 1.f);
  ASSERT_FLOAT_EQ(qt1.x, 0.f);
  ASSERT_FLOAT_EQ(qt1.y, 0.f);
  ASSERT_FLOAT_EQ(qt1.z, 0.f);

  Quaternion qt2 (5, 6, 7, 8);
  ASSERT_FLOAT_EQ(qt2.w, 5.f);
  ASSERT_FLOAT_EQ(qt2.x, 6.f);
  ASSERT_FLOAT_EQ(qt2.y, 7.f);
  ASSERT_FLOAT_EQ(qt2.z, 8.f);

  Quaternion qt3 (M_PI / 2, 0, 0);
  ASSERT_FLOAT_EQ(qt3.w, 1 / sqrt(2));
  ASSERT_FLOAT_EQ(qt3.x, 1 / sqrt(2));
  ASSERT_FLOAT_EQ(qt3.y, 0.f);
  ASSERT_FLOAT_EQ(qt3.z, 0.f);

  Quaternion qt4 (Vec3<float>(1,0,0), Vec3<float>(0,0,1), M_PI / 2);
  ASSERT_FLOAT_EQ(qt4.rotAngleInDeg(), 120.f);
}

TEST(quaternion_tests, quaternion_rollpitchyaw)
{
  Quaternion qt1 (0.1, 0, 0);
  ASSERT_FLOAT_EQ(qt1.getRoll(), 0.1f);

  Quaternion qt2 (0, 0.2, 0);
  ASSERT_FLOAT_EQ(qt2.getPitch(), 0.2f);

  Quaternion qt3 (0, 0, 0.3);
  ASSERT_FLOAT_EQ(qt3.getYaw(), 0.3f);
}

TEST(quaternion_tests, quaternion_inverse)
{
  Quaternion qt (M_PI / 4, 0, 0);
  Quaternion invQt = qt.inverse();

  ASSERT_FLOAT_EQ(invQt.getRoll(), - M_PI / 4);
}

TEST(quaternion_tests, quaternion_quaternion_multiplication)
{
  Quaternion qt1 (M_PI / 4, 0, 0);
  Quaternion qt2 (- M_PI / 2, 0, 0);
  Quaternion qt3 (0, M_PI / 2, 0);
  Quaternion result1 = qt1 * qt2;
  Quaternion result2 = qt2 * qt1;
  Quaternion result3 = qt1 * qt3;
  Quaternion result4 = qt3 * qt1;

  result3.normalize();

  ASSERT_FLOAT_EQ(result1.getRoll(), - M_PI / 4);
  ASSERT_FLOAT_EQ(result2.getRoll(), - M_PI / 4);
  ASSERT_FLOAT_EQ(result3.getRoll(), 0);
  ASSERT_FLOAT_EQ(result3.getYaw(), M_PI / 4);
  ASSERT_FLOAT_EQ(result3.getPitch(), M_PI / 2);
  ASSERT_FLOAT_EQ(result4.getRoll(), 0);
}

TEST(quaternion_tests, quaternion_scalar_multiplication)
{
  Quaternion qt (M_PI / 4, 0, 0);
  Quaternion scaledQt = qt * 2.f;
  ASSERT_FLOAT_EQ(scaledQt.w, qt.w * 2.f);
  ASSERT_FLOAT_EQ(scaledQt.x, qt.x * 2.f);
  ASSERT_FLOAT_EQ(scaledQt.y, qt.y * 2.f);
  ASSERT_FLOAT_EQ(scaledQt.z, qt.z * 2.f);

  scaledQt.normalize();

  //ASSERT_FLOAT_EQ(scaledQt.getRoll(), M_PI / 2);
}

TEST(pid_tests, pid_ctor)
{
  Pid pid(1.5f,2.3f,6.23f);
  ASSERT_EQ(pid.coeffP_, 1.5f);
  ASSERT_EQ(pid.coeffI_, 2.3f);
  ASSERT_EQ(pid.coeffD_, 6.23f);
}

TEST(pid_tests, pid_functor)
{
  Quaternion qt1 (1, 0, 0, 0);
  Quaternion qt2 (M_PI / 2, 0, 0);
  Quaternion qt3 (M_PI / 4, 0, 0);
  Quaternion qt4 (0, M_PI / 10, 0);
  Quaternion qt5 (0, M_PI / 5, 0);

  // test if it does nothing correctly
  Pid pid1(1,1,1);
  Vec3<float> ret = pid1(qt1, qt1);
  ASSERT_EQ(ret.x, 0.0f);
  ASSERT_EQ(ret.y, 0.0f);
  ASSERT_EQ(ret.z, 0.0f);

  // 90 degerees rotation error
  // alpha => z, beta => x, gamma => y
  Pid pid2(1,0,0);
  ret = pid2(qt1, qt2);
  ASSERT_FLOAT_EQ(ret.z, M_PI / 2);
  ASSERT_FLOAT_EQ(pid2.lastError.rotAngleInDeg(), 90.f);
  ASSERT_FLOAT_EQ(pid2.integError.rotAngleInDeg(), 90.f);
  ret = pid2(qt3, qt2);

  // incremental correction
  ASSERT_FLOAT_EQ(pid2.lastError.rotAngleInDeg(), 45.f);
  ASSERT_FLOAT_EQ(pid2.integError.rotAngleInDeg(), 135.f);

  // 45 degree error
  Pid pid3(1,1,1);
  ret = pid3(qt1, qt4);
  ASSERT_FLOAT_EQ(ret.x, M_PI / 10 * 3); // 3 times the current error
  ASSERT_FLOAT_EQ(pid3.lastError.rotAngleInDeg(), 18.f);
  ASSERT_FLOAT_EQ(pid3.integError.rotAngleInDeg(), 18.f);

  ret = pid3(qt1, qt5);
  ASSERT_FLOAT_EQ(ret.x, M_PI / 10 * 6); // 3 times the current error
  ASSERT_FLOAT_EQ(pid3.lastError.rotAngleInDeg(), 36.f);
  ASSERT_FLOAT_EQ(pid3.integError.rotAngleInDeg(), 3 * 18.f);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}