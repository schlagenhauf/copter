/*
 * Copyright 2014 by Jonas Schlagenhauf
 */

#ifndef GEOMETRY_VEC3_H_
#define GEOMETRY_VEC3_H_

namespace Geometry
{
  template <typename T>
  struct Vec3
  {
    T x, y, z;

    // Creates an empty vector
    Vec3() : x((T)0), y((T)0), z((T)0) {}

    // Initializes the vector with values
    Vec3(T inX, T inY, T inZ) : x(inX), y(inY), z(inZ) {}

    // Returns the squared norm
    inline float norm2() const { return x*x + y*y + z*z; }

    // Returns the dot product
    template <typename U>
    inline U dot(Vec3<U> const& v) const { return x*v.x + y*v.y + z*v.z; }

    // Returns the cross product. The returned vector has the same type as the
    // argument
    template <typename U>
    inline Vec3<U> cross(Vec3<U> const& v) const
    {
      Vec3<U> retVal;
      retVal.x = y*v.z - z*v.y;
      retVal.y = z*v.x - x*v.z;
      retVal.z = x*v.y - y*v.x;
      return retVal;
    }
  };
}

#endif  // GEOMETRICS_VEC3_H_
