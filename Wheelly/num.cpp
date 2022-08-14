#import "num.h"

Quaternion::Quaternion() {
  _i = _j = _k = 0;
  _w = 1;
}

Quaternion::Quaternion(const float w, const float i, const float j, const float k)
  : _i(i), _j(j), _k(k), _w(w) {
}

Quaternion& Quaternion::operator+=(const Quaternion& v) {
  _i += v._i;
  _j += v._j;
  _k += v._k;
  _w += v._w;
  return *this;
}

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

const Quaternion Quaternion::operator*(const Quaternion& v) const {
  float w = _w * v._w - _i * v._i - _j * v._j - _k * v._k;
  float i = _w * v._i + _i * v._w + _j * v._k - _k * v._j;
  float j = _w * v._j + _j * v._w + _k * v._i - _i * v._k;
  float k = _w * v._k + _k * v._w + _i * v._j - _j * v._i;
  return Quaternion(w, i, j, k);
}

const float Quaternion::norm2() const {
  return _w * _w + _i * _i + _j * _j + _k * _k;
}

const float Quaternion::norm() const {
  return sqrtf(norm2());
}

const Quaternion Quaternion::unit() const {
  float n = norm();
  return Quaternion(_w / n, _i / n, _j / n, _k / n);
}

const Quaternion Quaternion::conj() const {
  return Quaternion(_w, -_i, -_j, -_k);
}

const Vector3 Quaternion::euler() const {
  float t0 = 2 * (_w * _i + _j * _k);
  float t1 = 1 - 2 * (_i * _i + _j * _j);
  float phi = atan2f(t0, t1);

  float t2 = 2 * (_w * _j - _k * _i);
  t2 = min(max(t2, -1), 1);
  float theta = asinf(t2);

  float t3 = 2 * (_w * _k + _i * _j);
  float t4 = 1 - 2 * (_j * _j + _k * _k);
  float psi = atan2f(t3, t4);

  return Vector3(phi, theta, psi);
}

const Vector3 Quaternion::operator*(const Vector3& v) const {
  return (*this * Quaternion::pure(v) * conj()).vector();
}

const float Quaternion::yaw() const {
  return -atan2f(2 * (_i * _j + _w * _k),  _w * _w + _i * _i - _j * _j - _k * _k);
}

const float Quaternion::pitch() const {
  return -asinf(-2 * (_i * _k - _w * _j));
}

const float Quaternion::roll() const {
  return atan2f(2 * (_j * _k + _w * _i), _w * _w - _i * _i - _j * _j + _k * _k);
}

const Vector3 Quaternion::ypr() const {
  return Vector3(yaw(), pitch(), roll());
}

static const Quaternion Quaternion::rot(const Vector3& v) {
  float alpha = v.norm();
  float s = sinf(alpha / 2) / alpha;
  return Quaternion(cosf(alpha / 2), s * v[0], s * v[1], s * v[2]);
}

static const Quaternion Quaternion::rot(const float theta, const Vector3& axis) {
  Vector3 ax = axis.unit();
  float sp2 = sinf(theta / 2);
  return Quaternion(cosf(theta / 2), sp2 * ax[0], sp2 * ax[1], sp2 * ax[2]);
}

static const Quaternion Quaternion::rotX(const float theta) {
  return Quaternion(cosf(theta / 2), sinf(theta / 2), 0, 0);
}

static const Quaternion Quaternion::rotY(const float theta) {
  return Quaternion(cosf(theta / 2), 0, sinf(theta / 2), 0);
}

static const Quaternion Quaternion::rotZ(const float theta) {
  return Quaternion(cosf(theta / 2), 0, 0, sinf(theta / 2));
}

static const Quaternion Quaternion::pure(const Vector3& vect) {
  return Quaternion(0, vect[0], vect[1], vect[2]);
}

static const Quaternion Quaternion::fromEuler(const Vector3& euler) {
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

void Quaternion::print(const int prec = 2, Print& pr = Serial) const {
  pr.print(F("("));
  pr.print(_w, prec);
  pr.print(F(", "));
  pr.print(_i, prec);
  pr.print(F(", "));
  pr.print(_j, prec);
  pr.print(F(", "));
  pr.print(_k, prec);
  pr.print(F(")"));
}


Vector3::Vector3() {
  _x[0] = _x[1] = _x[2] = 0;
}

Vector3::Vector3(const float x, const float y, const float z) {
  _x[0] = x;
  _x[1] = y;
  _x[2] = z;
}

Vector3& Vector3::operator+=(const Vector3& v) {
  _x[0] += v._x[0];
  _x[1] += v._x[1];
  _x[2] += v._x[2];
  return *this;
}

const Vector3 Vector3::operator-(const Vector3& v) const {
  return Vector3(_x[0] - v._x[0],
                 _x[1] - v._x[1],
                 _x[2] - v._x[2]);
}

const Vector3 Vector3::operator*(const float scale) const {
  return Vector3(_x[0] * scale,
                 _x[1] * scale,
                 _x[2] * scale);
}

const float Vector3::operator*(const Vector3& v) const {
  return _x[0] * v[0] + _x[1] * v[1] + _x[2] * v[2];
}

const Vector3 Vector3::operator/(const float divisor) const {
  return Vector3(_x[0] / divisor,
                 _x[1] / divisor,
                 _x[2] / divisor);
}

Vector3& Vector3::operator*=(const float v) {
  _x[0] *= v;
  _x[1] *= v;
  _x[2] *= v;
  return *this;
}

Vector3& Vector3::operator/=(const float divider) {
  _x[0] /= divider;
  _x[1] /= divider;
  _x[2] /= divider;
  return *this;
}

const float Vector3::norm2() const {
  return _x[0] * _x[0] + _x[1] * _x[1] + _x[2] * _x[2];
}

const float Vector3::norm() const {
  return sqrtf(norm2());
}

const Vector3 Vector3::unit() const {
  float n = norm();
  return Vector3(_x[0] / n, _x[1] / n, _x[2] / n);
}

const Vector3 Vector3::cross(const Vector3& v) const {
  return Vector3(
           _x[1] * v._x[2] - _x[2] * v._x[1],
           _x[2] * v._x[0] - _x[0] * v._x[2],
           _x[0] * v._x[1] - _x[1] * v._x[0]);
}

void Vector3::print(int prec, Print& pr) const {
  pr.print(F("("));
  pr.print(_x[0], prec);
  pr.print(F(", "));
  pr.print(_x[1], prec);
  pr.print(F(", "));
  pr.print(_x[2], prec);
  pr.print(F(")"));
}
