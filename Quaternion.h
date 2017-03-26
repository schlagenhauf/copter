/**
 * @file    Quaternion
 * @author  Jonas Schlagenhauf <pho3nyx@web.de>
 * @version 0.1.0
 *
 * @copyright Copyright 6 March 2014 Jonas Schlagenhauf
 *
 * @section LICENSE
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details at
 * http://www.gnu.org/copyleft/gpl.html
 *
 * @section DESCRIPTION
 *
 * A Quaternion class with useful constructors from various types and handy
 * conversions.
 * A Quaternion is a complex value in 4D space
 * (q = q0 + q*i + q2*j + q3*k, i*j*k = -1)
 * A normalized quaternion (unit quaternion) can be used to efficiently and
 * elegantly represent rotations. An advantage over euler angles is the
 * avoidance of gimbal locks (and no ugly case distinctions).
 * It also needs less operations per operation than a matrix rotation.
 *
 * Wherever axes are needed, +x * denotes "forward" and the roll rotation axis,
 * +y denotes "right" and the pitch rotation axis, and +z "downward" and the yaw
 * rotation axis.
 *
 * REMARK:
 * Due to performance reasons, this class does no checking for you whether you use it correctly.
 * In particular this means it is up to you to normalize the quaternion when necessary.
 */


#ifndef GEOMETRY_QUATERNION_H_
#define GEOMETRY_QUATERNION_H_

#define _USE_MATH_DEFINES

#ifdef _GTEST_ON
#include <gtest/gtest.h>
#endif
#include <math.h>
#include <cmath>
#include <cstring>
#include "./Vec3.h"

namespace Geometry
{
  // Quaternion class with common operation methods. Keep in mind that almost all
  // methods require a normalized quaternion (unit quaternion) to operate on.
  class Quaternion
  {
   public:
    double w, x, y, z;
    // Standard constructor for the lazy
    Quaternion() : w(1), x(0), y(0), z(0) {}
    // Direct initialization constructor
    Quaternion(double inW, double inX, double inY, double inZ)
      : w(inW), x(inX), y(inY), z(inZ) {}
    // Creates a quaternion based on the three euler angles
    Quaternion(double alpha, double beta, double gamma);
    // Creates a quaternion based on the angle around a rotation axis
    template <typename T>
    Quaternion(double angle, Vec3<T> const& axis);
    // Creates a quaternion which represents the rotation between two unit vectors
    template <typename T, typename U>
    Quaternion(Vec3<T> const& v1, Vec3<U> const& v2);
    // Creates a quaternion which represents the rotation between two unit
    // vectors with an additional rotation around the vector axis
    template <typename T, typename U>
    Quaternion(Vec3<T> const& v1, Vec3<U> const& v2, double axialRotation);

    void operator=(Quaternion const& rOp);
    // Quaternion multiplication. Results in an addition of the underlying
    // rotations
    Quaternion operator*(Quaternion const& rOp) const;
    // Quaternion scaling
#ifdef _GTEST_ON
    FRIEND_TEST(quaternion_tests, quaternion_scalar_multiplication);
#endif
    Quaternion operator*(double const& rOp) const;
    // Quaternion addition
    Quaternion operator+(Quaternion const& rOP) const;

    // Returns the inverse
    Quaternion inverse() const;

    // Normalizes the 4 elements
    void normalize();

    // Checks if the quaternion is normalized
    bool isNormalized() const;
    // Angle between this and another quaternion in 4D space (has nothing to do
    // with the rotation represented by the quaternions
    double angle(Quaternion const& toQuat) const;
    // Spherical linear interpolation to a destination quaternion.
    Quaternion slerp(Quaternion const& destQt, double t, double eps = 0.01) const;
    // Linear interpolation to a destination quaternion
    Quaternion lerp(Quaternion const& destQt, double t) const;
    // Packs everything in a byte array. Requires a preallocated byte array of
    // size 16
    void toByteArray(char* bArray) const;
    // Return the rotation in degrees
    double rotAngleInDeg();

    double getRoll() const;
    double getPitch() const;
    double getYaw() const;

    static float fastSqrt(float number);
  };

  /**
   *
   * Constructor from Euler angles. (I have too re-check the angle sequence
   * sometimes)
   *
   * @param alpha Rotation around the z axis (yaw)
   * @param beta Rotation around the y axis (pitch)
   * @param gamma Rotation around the x axis (roll)
   */
  Quaternion::Quaternion(double alpha, double beta, double gamma)
  {
    double ca = cos(alpha/2);
    double sa = sin(alpha/2);
    double cb = cos(beta/2);
    double sb = sin(beta/2);
    double cg = cos(gamma/2);
    double sg = sin(gamma/2);
    this->w = ca*cb*cg+sa*sb*sg;
    this->x = sa*cb*cg-ca*sb*sg;
    this->y = ca*sb*cg+sa*cb*sg;
    this->z = ca*cb*sg-sa*sb*cg;
  }

  /**
   *
   * Constructor from angle and rotation axis
   *
   * @param angle Rotation magnitude
   * @param gamma Rotation axis
   */
  template <typename T>
  Quaternion::Quaternion(double angle, Vec3<T> const& axis)
  {
    this->w = cos(angle/2);
    this->x = axis.x * sin(angle/2);
    this->y = axis.y * sin(angle/2);
    this->z = axis.z * sin(angle/2);
  }

  /**
   *
   * Constructor from two vectors. The resulting quaternion represents the
   * rotation between the vectors.
   *
   * @param v1 First vector
   * @param v2 Second vector
   */
  template <typename T, typename U>
  Quaternion::Quaternion(Vec3<T> const& v1, Vec3<U> const& v2)
  {
    this->w = sqrt(v1.norm2() * v2.norm2()) + v1.dot(v2);
    Vec3<U> axis = v1.cross(v2);
    this->x = axis.x;
    this->y = axis.y;
    this->z = axis.z;
  }

  /**
   *
   * Constructor from two vectors. The resulting quaternion represents the
   * rotation between the vectors.
   *
   * @param v1 First vector
   * @param v2 Second vector
   * @param rotation along the vector axis
   */
  template <typename T, typename U>
  Quaternion::Quaternion(Vec3<T> const& v1, Vec3<U> const& v2, double axialRotation)
  {
    this->w = sqrt(v1.norm2() * v2.norm2()) + v1.dot(v2);
    Vec3<U> axis = v1.cross(v2);
    this->x = axis.x;
    this->y = axis.y;
    this->z = axis.z;

    *this = *this * Quaternion(0, 0, axialRotation);
  }

  /**
   * Assigns the values of the right hand side quaternion
   *
   * @param rOp Right hand side operand (Quaternion)
   */
  void Quaternion::operator=(Quaternion const& rOp)
  {
    this->w = rOp.w;
    this->x = rOp.x;
    this->y = rOp.y;
    this->z = rOp.z;
  }

  /**
   *
   * Quaternion Multiplication Operator. Multiplication of two quaternions
   * corresponds to a combined resulting rotation. Note that a quaternion
   * multiplication is non-commutative.
   * It is necessary to normalize the quaternion beforehand!
   *
   * @param rOp Right hand side operand (Quaternion)
   * @return A new quaternion.
   */
  Quaternion Quaternion::operator*(Quaternion const& rOp) const
  {
    Quaternion retVal;
    retVal.w = w * rOp.w - x * rOp.x - y * rOp.y - z * rOp.z;
    retVal.x = w * rOp.x + x * rOp.w + y * rOp.z - z * rOp.y;
    retVal.y = w * rOp.y - x * rOp.z + y * rOp.w + z * rOp.x;
    retVal.z = w * rOp.z + x * rOp.y - y * rOp.x + z * rOp.w;
    return retVal;
  }

  /**
   *
   * Scalar Multiplication Operator. Multiplication of a quaternion with a
   * scalar, corresponds to a scaling of the represented rotation.
   * It is necessary to normalize the quaternion beforehand!
   *
   * @param rOp Right hand side operand (double)
   * @return A new quaternion.
   */
  Quaternion Quaternion::operator*(double const& rOp) const
  {
    return Quaternion ( rOp == 0? 1 : w * rOp, x * rOp, y * rOp, z * rOp);
  }

  /**
   *
   * Quaternion Addition Operator. Addition of two Quaternions does NOT result
   * in an addition of the respective rotations. Read up quaternions!
   * It is necessary to normalize the quaternion beforehand!
   *
   * @param rOp Right hand side operand (Quaternion)
   * @return A new quaternion.
   */
  Quaternion Quaternion::operator+(Quaternion const& rOp) const
  {
    return Quaternion(w + rOp.w, x + rOp.x, y + rOp.y, z + rOp.z);
  }

  /*
   * Returns the inverse of the underlying quaternion as a new instance.
   */
  Quaternion Quaternion::inverse() const
  {
    return Quaternion(w, -x, -y, -z);
  }

  /**
   *
   * Normalizes the Quaternion in place (not a copy that is returned).
   * This is necessary for almost all quaternion operations before executing.
   */
  void Quaternion::normalize()
  {
    double norm = sqrt(w*w + x*x + y*y + z*z);
    this->w /= norm;
    this->x /= norm;
    this->y /= norm;
    this->z /= norm;
  }

  /**
   *
   * Returns whether the Quaternion is normalized.
   *
   * @return True, if normalized.
   */
  bool Quaternion::isNormalized() const
  {
    return w*w + x*x + y*y + z*z == 1;
  }

  /**
   *
   * Calculates the angle between the given and the underlying quaternion in 4D
   * space. Has nothing to do with rotations in 3D space.
   *
   * @param toQuat The quaternion to which the angle is calculated
   * @return The angle between the two quaternions
   */
  double Quaternion::angle(Quaternion const& toQuat) const
  {
    return acos(w*toQuat.w + x*toQuat.x + y*toQuat.y + z*toQuat.z);
  }

  /**
   *
   * Computes a [s]pherical [l]inear int[erp]olation between the given and the
   * underlying quaternion and returns the resulting rotation as a new
   * quaternion.
   * It is necessary to normalize the quaternion beforehand!
   *
   * @param destQt The quaternion of the other side of the interpolation
   * @param t "Time", the interpolation value between 0 and 1
   * @param eps Angular threshold where to begin with LERP
   * @return The resulting rotation as a quaternion
   */
  Quaternion Quaternion::slerp(Quaternion const& destQt,
                               double t, double eps) const
  {
    if (t <= 0)
      return *this;
    if (t >= 1)
      return destQt;

    double fac = 1;
    double ang = this->angle(destQt);
    if (std::abs(ang) < eps)
    {
      return lerp(destQt, t);
    }
    else if (ang < 0)
    {
      ang = -ang;
      fac = -1;
    }

    double fromWgt = sin((1-t) * ang);
    double toWgt = sin(t * ang) * fac;
    double denom = sin(ang);

    Quaternion retVal(
        (this->w * fromWgt + destQt.w * toWgt) / denom,
        (this->x * fromWgt + destQt.x * toWgt) / denom,
        (this->y * fromWgt + destQt.y * toWgt) / denom,
        (this->z * fromWgt + destQt.z * toWgt) / denom);
    return retVal;
  }

  /**
   *
   * Computes a [l]inear int[erp]olation between the given and the underlying
   * quaternion and returns the resulting rotation as a new quaternion.
   * This method is mainly used by SLERP, usually there is no application where
   * to call it manually.
   * It is necessary to normalize the quaternion beforehand!
   *
   * @param destQt The quaternion on the other side of the interpolation
   * @param t "Time", the interpolation value between 0 and 1
   * @return The resulting rotation as a quaternion
   */
  Quaternion Quaternion::lerp(Quaternion const& destQt, double t) const
  {
    Quaternion retVal(
        this->w * (1-t) + destQt.w * t,
        this->x * (1-t) + destQt.x * t,
        this->y * (1-t) + destQt.y * t,
        this->z * (1-t) + destQt.z * t);
    return retVal;
  }

  /**
   *
   * Serializes the quaternion. Make sure to allocate enough space for four
   * doubles.
   *
   * @param bArray The byte array to be filled
   */
  void Quaternion::toByteArray(char* bArray) const
  {
    float fW = (float)w;
    float fX = (float)x;
    float fY = (float)y;
    float fZ = (float)z;
    memcpy(bArray, &fW, 4);
    memcpy(bArray+4, &fX, 4);
    memcpy(bArray+8, &fY, 4);
    memcpy(bArray+12, &fZ, 4);
  }

  /**
   *
   * Returns the angle of the rotation represented by the quaternion.
   * It is necessary to normalize the quaternion beforehand!
   *
   * @return The angle of the rotation.
   */
  double Quaternion::rotAngleInDeg()
  {
    normalize();
    return 2*acos(w) * 180 / M_PI;
  }

  // WARNING: Pitch and Roll switched to get correct output.
  // Error may reside elsewhere
  double Quaternion::getPitch() const
  {
    return atan2(2 * y * w - 2 * x * z, 1 - 2 * y * y - 2 * z * z);
  }

  double Quaternion::getYaw() const
  {
    return asin(2 * x * y + 2 * z * w);
  }

  double Quaternion::getRoll() const
  {
    return atan2(2 * x * w - 2 * y * z, 1 - 2 * x * x - 2 * z * z);
  }

  /*
  float Quaternion::fastSqrt(float number)
  {
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y  = number;
    i  = * ( long * ) &y;                       // evil floating point bit level hacking
    i  = 0x5f3759df - ( i >> 1 );               // what the fuck?
    y  = * ( float * ) &i;
    y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
    y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

    return y;
  }
  */
}
#endif  // GEOMETRICS_QUATERNION_H_
