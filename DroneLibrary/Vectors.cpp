#include "Vectors.h"
#include <Arduino.h>

template<typename T>
float vector3<T>::magnitude() {
    return sqrt((*this) * (*this));
}

template<typename T>
template<typename U>
vector3<T>& vector3<T>::operator=(const vector3<U>& other) {
    x = other.x;
    y = other.y;
    z = other.z;
    return *this;
}

template<typename T, typename U>
vector3<T> operator+(const vector3<T>& u, const vector3<U>& v) {
    return {u.x + v.x, u.y + v.y, u.z + v.z};
}

template<typename T, typename U>
vector3<T>& operator+=(vector3<T>& u, const vector3<U>& v) {
    u.x += v.x;
    u.y += v.y;
    u.z += v.z;
    return u;
}

template<typename T, typename U>
vector3<T> operator-(const vector3<T>& u, const vector3<U>& v) {
    return {u.x - v.x, u.y - v.y, u.z - v.z};
}

template<typename T, typename U>
vector3<T>& operator-=(vector3<T>& u, const vector3<U>& v) {
    u.x -= v.x;
    u.y -= v.y;
    u.z -= v.z;
    return u;
}

template<typename T, typename U>
float operator*(const vector3<T>& u, const vector3<U>& v) {
    return u.x * v.x + u.y * v.y + u.z * v.z;
}

template<typename T>
vector3<T> operator*(const vector3<T>& u, float a) {
    return {u.x * a, u.y * a, u.z * a};
}

template<typename T>
vector3<T>& operator*=(vector3<T>& u, float a) {
    u.x *= a;
    u.y *= a;
    u.z *= a;
    return u;
}

template<typename T>
vector3<T> operator/(const vector3<T>& u, float a) {
    if (a == 0)
        return {0, 0, 0};
    return {u.x / a, u.y / a, u.z / a};
}

template<typename T>
vector3<T>& operator/=(vector3<T>& u, float a) {
    if (a == 0)
        return {0, 0, 0};
    u.x /= a;
    u.y /= a;
    u.z /= a;
    return u;
}

template<typename T, typename U>
vector3<T> addVectors(const vector3<T>& a, const vector3<U>& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

template<typename T, typename U>
vector3<T> subtractVectors(const vector3<T>& a, const vector3<U>& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

template<typename T, typename U>
float dotProduct(const vector3<T>& a, const vector3<U>& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

template<typename T>
float vectorMagnitude(const vector3<T>& a) {
    return sqrt(a * a);
}

template<typename T, typename U>
vector3<T> projectVector(const vector3<T>& a, const vector3<U>& b) {
    return b * ((a * b) / (b * b));
}