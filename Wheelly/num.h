#ifndef num_h
#define num_h

#include "Arduino.h"

class Vector3 {
  public:
    Vector3();
    Vector3(const float x, const float y, const float z);

    const float operator[](const int i) const {
      return _x[i];
    }
    Vector3& operator+=(const Vector3& v);
    Vector3& operator*=(const float v);
    Vector3& operator/=(const float v);
    
    const Vector3 operator-(const Vector3& v) const;
    const Vector3 operator*(const float scale) const;
    const float operator*(const Vector3& v) const;
    const Vector3 operator/(const float divisor) const;

    const Vector3 cross(const Vector3& v) const;

    const float norm2() const;
    const float norm() const;
    const Vector3 unit() const;
    void print(const int prec = 2, Print& pr = Serial) const;

  private:
    float _x[3];
};

class Quaternion {
  public:
    static const Quaternion fromEuler(const Vector3& euler);
    static const Quaternion rot(const Vector3& rotation);
    static const Quaternion rot(const float theta, const Vector3& axis);
    static const Quaternion rotX(const float theta);
    static const Quaternion rotY(const float theta);
    static const Quaternion rotZ(const float theta);
    static const Quaternion pure(const Vector3& vect);

    Quaternion();
    Quaternion(const float w, const float i, const float j, const float k);
    Quaternion& operator+=(const Quaternion& v);
    Quaternion& operator*=(const Quaternion& v);
    const Quaternion operator*(const Quaternion& v) const;
    const Vector3 operator*(const Vector3& v) const;
    const float norm2() const;
    const float norm() const;
    const float yaw() const;
    const float pitch() const;
    const float roll() const;
    const Vector3 ypr() const;
    const Quaternion unit() const;
    const Quaternion conj() const;
    const Vector3 euler() const;
    const Vector3 vector() const {return Vector3(_i, _j, _k);}
    const float i() const {
      return _i;
    }
    const float j() const {
      return _j;
    }
    const float k() const {
      return _k;
    }
    const float w() const {
      return _w;
    }
    void print(const int prec = 2, Print& pr = Serial) const;

  private:
    float _i;
    float _j;
    float _k;
    float _w;
};

#endif
