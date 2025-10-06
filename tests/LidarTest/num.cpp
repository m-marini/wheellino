/*
 * Copyright (c) 2023  Marco Marini, marco.marini@mmarini.org
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 *    END OF TERMS AND CONDITIONS
 *
 */

#include <Arduino.h>
#include <stdio.h>
#import "num.h"

/*
   Adds in place the quaternion
   @param v the quaternion
*/
Quaternion& Quaternion::operator+=(const Quaternion& v) {
  _i += v._i;
  _j += v._j;
  _k += v._k;
  _w += v._w;
  return *this;
}

/*
   Multiplies in place the quaternion
   @param v the quaternion
*/
Quaternion& Quaternion::operator*=(const Quaternion& v) {
  float w = _w * v._w - _i * v._i - _j * v._j - _k * v._k;
  float i = _w * v._i + _i * v._w + _j * v._k - _k * v._j;
  float j = _w * v._j + _j * v._w + _k * v._i - _i * v._k;
  float k = _w * v._k + _k * v._w + _i * v._j - _j * v._i;
  _w = w;
  _i = i;
  _j = j;
  _k = k;
  return *this;
}

/*
   Returns the product of two quaternions
   @param v the quaternion
*/
const Quaternion Quaternion::operator*(const Quaternion& v) const {
  float w = _w * v._w - _i * v._i - _j * v._j - _k * v._k;
  float i = _w * v._i + _i * v._w + _j * v._k - _k * v._j;
  float j = _w * v._j + _j * v._w + _k * v._i - _i * v._k;
  float k = _w * v._k + _k * v._w + _i * v._j - _j * v._i;
  return Quaternion(w, i, j, k);
}

/*
   Returns the square norm of quaternion
*/
const float Quaternion::norm2() const {
  return _w * _w + _i * _i + _j * _j + _k * _k;
}

/*
   Returns the norm of quaternion
*/
const float Quaternion::norm() const {
  return sqrtf(norm2());
}

/*
   Returns the unit quaternion
*/
const Quaternion Quaternion::unit() const {
  float n = norm();
  return Quaternion(_w / n, _i / n, _j / n, _k / n);
}

/*
   Returns the conjugate of quaternion
*/
const Quaternion Quaternion::conj() const {
  return Quaternion(_w, -_i, -_j, -_k);
}

/*
   Returns the euler vector
*/
const Vector3 Quaternion::euler() const {
  float t0 = 2 * (_w * _i + _j * _k);
  float t1 = 1 - 2 * (_i * _i + _j * _j);
  float phi = atan2f(t0, t1);

  float t2 = 2 * (_w * _j - _k * _i);
  t2 = min(max(t2, (float)(-1)), (float)1);
  float theta = asinf(t2);

  float t3 = 2 * (_w * _k + _i * _j);
  float t4 = 1 - 2 * (_j * _j + _k * _k);
  float psi = atan2f(t3, t4);

  return Vector3(phi, theta, psi);
}

/*
   Returns the product of quaternion by vector
*/
const Vector3 Quaternion::operator*(const Vector3& v) const {
  return (*this * Quaternion::pure(v) * conj()).vector();
}

/*
   Returns the yaw angle of quaternion (RAD)
*/
const float Quaternion::yaw() const {
  return -atan2f(2 * (_i * _j + _w * _k), _w * _w + _i * _i - _j * _j - _k * _k);
}

/*
   Returns the pitch angle of quaternion (RAD)
*/
const float Quaternion::pitch() const {
  return -asinf(-2 * (_i * _k - _w * _j));
}

/*
  Returns the roll abgle of quaternion (RAD)
*/
const float Quaternion::roll() const {
  return atan2f(2 * (_j * _k + _w * _i), _w * _w - _i * _i - _j * _j + _k * _k);
}

/*
   Returns the (yaw, pitch, roll) angles vector
*/
const Vector3 Quaternion::ypr() const {
  return Vector3(yaw(), pitch(), roll());
}

/*
   Returns the quaternion of rotation vector
   @param v the rotation vector (rotation axis by angle RAD)
*/
const Quaternion Quaternion::rot(const Vector3& v) {
  float alpha = v.norm();
  float s = sinf(alpha / 2) / alpha;
  return Quaternion(cosf(alpha / 2), s * v[0], s * v[1], s * v[2]);
}

/*
   Returns the quaternion of rotation vector
   @param theta the rotatino angle (RAD)
   @param axis the rotation rotation axis
*/
const Quaternion Quaternion::rot(const float theta, const Vector3& axis) {
  Vector3 ax = axis.unit();
  float sp2 = sinf(theta / 2);
  return Quaternion(cosf(theta / 2), sp2 * ax[0], sp2 * ax[1], sp2 * ax[2]);
}

/*
   Returns the quaternion of rotation round X axis
   @param theta the angle (RAD)
*/
const Quaternion Quaternion::rotX(const float theta) {
  return Quaternion(cosf(theta / 2), sinf(theta / 2), 0, 0);
}

/*
   Returns the quaternion of rotation round Y axis
   @param theta the angle (RAD)
*/
const Quaternion Quaternion::rotY(const float theta) {
  return Quaternion(cosf(theta / 2), 0, sinf(theta / 2), 0);
}

/*
   Returns the quaternion of rotation round Z axis
   @param theta the angle (RAD)
*/
const Quaternion Quaternion::rotZ(const float theta) {
  return Quaternion(cosf(theta / 2), 0, 0, sinf(theta / 2));
}

/*
   Returns the pure quaternion
   @param vect the vector
*/
const Quaternion Quaternion::pure(const Vector3& vect) {
  return Quaternion(0, vect[0], vect[1], vect[2]);
}

/*
   Returns the quaterion of euler rotation vector
   @param euler the euler vector
*/
const Quaternion Quaternion::fromEuler(const Vector3& euler) {
  float c0 = cosf(euler[0] / 2);
  float c1 = cosf(euler[1] / 2);
  float c2 = cosf(euler[2] / 2);
  float s0 = sinf(euler[0] / 2);
  float s1 = sinf(euler[1] / 2);
  float s2 = sinf(euler[2] / 2);
  return Quaternion(
    c0 * c1 * c2 + s0 * s1 * s2,
    s0 * c1 * c2 - c0 * s1 * s2,
    c0 * s1 * c2 + s0 * c1 * s2,
    c0 * c1 * s2 - s0 * s1 * c2);
}

/*
   Prints the quaternion
*/
const String Quaternion::toString(const int prec) const {
  char bfr[64];
  sprintf(bfr, "(%*f, %*f, %*f, %*f)",
          prec, (double)_w, prec, (double)_i, prec, (double)_j, prec, (double)_k);
  return String(bfr);
}

/*
   Adds in place the vector
   @param v the vector
*/
Vector3& Vector3::operator+=(const Vector3& v) {
  _x[0] += v._x[0];
  _x[1] += v._x[1];
  _x[2] += v._x[2];
  return *this;
}

/*
   Returns the difference of two vectors
   @param v the vector
*/
const Vector3 Vector3::operator-(const Vector3& v) const {
  return Vector3(_x[0] - v._x[0],
                 _x[1] - v._x[1],
                 _x[2] - v._x[2]);
}

/*
   Returns the product of vector by scale factor
   @param scale the scale factor
*/
const Vector3 Vector3::operator*(const float scale) const {
  return Vector3(_x[0] * scale,
                 _x[1] * scale,
                 _x[2] * scale);
}


/*
   Returns the scalar product of two vectors
   @param v the vector
*/
const float Vector3::operator*(const Vector3& v) const {
  return _x[0] * v[0] + _x[1] * v[1] + _x[2] * v[2];
}

/*
   Returns the division of vector by scale factor
   @param scale the scale factor
*/
const Vector3 Vector3::operator/(const float divisor) const {
  return Vector3(_x[0] / divisor,
                 _x[1] / divisor,
                 _x[2] / divisor);
}

/*
   Multiplies in place by scale factor
   @param v scale factor
*/
Vector3& Vector3::operator*=(const float v) {
  _x[0] *= v;
  _x[1] *= v;
  _x[2] *= v;
  return *this;
}

/*
   Divides in place by scale factor
   @param v scale factor
*/
Vector3& Vector3::operator/=(const float divider) {
  _x[0] /= divider;
  _x[1] /= divider;
  _x[2] /= divider;
  return *this;
}

/*
   Returns the square norm of vector (square length)
*/
const float Vector3::norm2() const {
  return _x[0] * _x[0] + _x[1] * _x[1] + _x[2] * _x[2];
}

/*
   Returns the norm of vector (length)
*/
const float Vector3::norm() const {
  return sqrtf(norm2());
}

/*
   Returns the unit vector
*/
const Vector3 Vector3::unit() const {
  float n = norm();
  return Vector3(_x[0] / n, _x[1] / n, _x[2] / n);
}

/*
   Returns the cross product of two vectors
   @param v the vector
*/
const Vector3 Vector3::cross(const Vector3& v) const {
  return Vector3(
    _x[1] * v._x[2] - _x[2] * v._x[1],
    _x[2] * v._x[0] - _x[0] * v._x[2],
    _x[0] * v._x[1] - _x[1] * v._x[0]);
}

/*
   Prints a vector
*/
const String Vector3::toString(const int prec) const {
  char msg[64];
  sprintf(msg, "(%*f, %*f, %*f)",
          prec, (double)_x[0], prec, (double)_x[1], prec, (double)_x[2]);
  return String(msg);
}
