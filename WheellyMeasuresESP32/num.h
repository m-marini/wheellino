#ifndef num_h
#define num_h

/*
  Numeric functino libriaries
*/

#include "Arduino.h"

/*
   Returns the normalized angle (RAD) (-PI ... PI)
   @param rad the angle (RAD)
*/
extern const float normalRad(const float rad);

/*
   Returns the normalized angle (DEG) (-180 ... 179)
   @param deg the angle (DEG)
*/
extern const int normalDeg(const int deg);

/*
   Returns the clipped value
   @param value the value
   @param min the minimum value
   @param max the maximum value
*/
extern const long clip(const long value, const long min, const long max);

/*
   3D vector
*/
class Vector3 {
  public:
    /*
       Creates the vector (0,0,0)
    */
    Vector3() {
      _x[0] = _x[1] = _x[2] = 0;
    }

    /*
       Creates the vector (x,y,z)
    */
    Vector3(const float x, const float y, const float z) {
      _x[0] = x;
      _x[1] = y;
      _x[2] = z;
    }

    /*
      Returns the coordinate of vector
      @param i the dimension index
    */
    const float operator[](const int i) const {
      return _x[i];
    }

    /*
       Adds in place a vector
       @param v the vector
    */
    Vector3& operator+=(const Vector3& v);

    /*
       Multiplies in place the vector by a scale factor
       @param v the scale factor
    */
    Vector3& operator*=(const float v);

    /*
       Divides in place the vector by a scale factor
       @param v the scale factor
    */
    Vector3& operator/=(const float v);

    /*
      Returns the difference of two vectors
    */
    const Vector3 operator-(const Vector3& v) const;

    /*
       Returns product of a vector by a scale factor
    */
    const Vector3 operator*(const float scale) const;

    /*
       Returns the scalar product of two vectors
    */
    const float operator*(const Vector3& v) const;

    /*
       Returns division of a vector by a scale factor
    */
    const Vector3 operator/(const float divisor) const;

    /*
       Returns cross product of two vectors
    */
    const Vector3 cross(const Vector3& v) const;

    /*
       Returns the square norma (square lenght) of a vector (lenght)
    */
    const float norm2() const;

    /*
       Returns the norma (lenght) of a vector (lenght)
    */
    const float norm() const;

    /*
       Returns the the unit vector
    */
    const Vector3 unit() const;

    /*
      Convert to string
    */
    const String toString(const int prec = 2) const;

  private:
    float _x[3];
};

/*
   Quaternion
*/
class Quaternion {
  public:
    /*
       Returns the quaternion from euler vector
       @param euler the euler vector
    */
    static const Quaternion fromEuler(const Vector3& euler);

    /*
       Returns the quaternion of rotation
       @param rotation the rotation vector (rotation axis by angle RAD)
    */
    static const Quaternion rot(const Vector3& rotation);

    /*
       Returns the quaternion of rotation
       @param theta the angle (RAD)
       @param axis the rotation axis
    */
    static const Quaternion rot(const float theta, const Vector3& axis);

    /*
       Returns the quaternion of rotation route X axis
       @param theta the angle (RAD)
    */
    static const Quaternion rotX(const float theta);

    /*
       Returns the quaternion of rotation route Y axis
       @param theta the angle (RAD)
    */
    static const Quaternion rotY(const float theta);

    /*
       Returns the quaternion of rotation route Z axis
       @param theta the angle (RAD)
    */
    static const Quaternion rotZ(const float theta);

    /*
       Returns the quaternion of pure rotation route Z axis
       @param vect the vector (?)
    */
    static const Quaternion pure(const Vector3& vect);

    /*
       Creates unary the quaternion (1,0,0,0)
    */
    Quaternion() : _w(1), _i(0), _j(0), _k(0) {}

    /*
       Creates the quaternion
       @param w the w coordinate
       @param i the i coordinate
       @param j the j coordinate
       @param k the k coordinate
    */
    Quaternion(const float w, const float i, const float j, const float k)
      : _i(i), _j(j), _k(k), _w(w) {
    }

    /*
       Adds in place the quaternion
       @param v the quaternion
    */
    Quaternion& operator+=(const Quaternion& v);

    /*
       Multiplies in place the quaternion
       @param v the quaternion
    */
    Quaternion& operator*=(const Quaternion& v);

    /*
      Multiplies two the quaternion
      @param v the quaternion
    */
    const Quaternion operator*(const Quaternion& v) const;

    /*
      Multiplies the quaternion by vector.
      Returns the rotated vector
      @param v the vector
    */
    const Vector3 operator*(const Vector3& v) const;

    /*
       Returns the square norm of quaternion
    */
    const float norm2() const;

    /*
       Returns the norm of quaternion
    */
    const float norm() const;

    /*
       Returns the yaw value of quaternion (RAD)
    */
    const float yaw() const;

    /*
       Returns the pitch value of quaternion (RAD)
    */
    const float pitch() const;

    /*
       Returns the roll value of quaternion (RAD)
    */
    const float roll() const;

    /*
       Returns the (yaw, pitch, roll) vector
    */
    const Vector3 ypr() const;

    /*
       Returns the unit vector
    */
    const Quaternion unit() const;

    /*
       Returns the conjugate of quaternion
    */
    const Quaternion conj() const;

    /*
       Returns the euler vector
    */
    const Vector3 euler() const;

    /*
       Returns the (i,j,k) vector
    */
    const Vector3 vector() const {
      return Vector3(_i, _j, _k);
    }

    /*
       Returns the i component of quaternion
    */
    const float i() const {
      return _i;
    }

    /*
       Returns the j component of quaternion
    */
    const float j() const {
      return _j;
    }

    /*
      Returns the k component of quaternion
    */
    const float k() const {
      return _k;
    }

    /*
       Returns the w component of quaternion
    */
    const float w() const {
      return _w;
    }

    /*
      Converts to string
    */
    const String toString(const int prec = 2) const;

  private:
    float _i;
    float _j;
    float _k;
    float _w;
};

/*
   Returns the fuzzy positivity of the value
   value <= 0       => 0
   value >= range   => 1
   @param value the value
   @param the range
*/
extern const float fuzzyPositive(const float value, const float range);


/*
   Generates a weighted value by avaraging the added values
*/
class Defuzzier {
  public:
    /*
       Creates the defuzzier
    */
    Defuzzier();

    /*
       Adds a weighted value
       @param value the value
       @param weight the weight
    */
    void add(const float value, const float weight);

    /*
       Resets the defuzzier
    */
    void reset();

    /*
       Returns the defizzied value
    */
    const float defuzzy() const;

  private:
    float _sum;
    float _scale;
};

#endif
