#ifndef VECTORS_H_
#define VECTORS_H_

#include <Arduino.h>

template<typename T>
struct vector3 {
    T x, y, z;

    float magnitude() {
        return sqrt((*this) * (*this));
    }

    template<typename U>
    vector3<T>& operator=(const vector3<U>& other) {
        x = other.x;
        y = other.y;
        z = other.z;
        return *this;
    }
};

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
    return {u.x / a, u.y / a, u.z / a};
}

template<typename T>
vector3<T>& operator/=(vector3<T>& u, float a) {
    u.x /= a;
    u.y /= a;
    u.z /= a;
    return u;
}

template<typename T>
vector3<T> operator-(const vector3<T>& u) {
    return {-u.x, -u.y, -u.z};
}

template<typename T>
vector3<T> crossProduct(const vector3<T>& u, const vector3<T>& v) {
    return {u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z, u.x * v.y - u.y * v.x};
}

template<typename T, typename U>
vector3<T> addVectors(const vector3<T>& a, const vector3<U>& b) {
    return a + b;
}

template<typename T, typename U>
vector3<T> subtractVectors(const vector3<T>& a, const vector3<U>& b) {
    return a - b;
}

template<typename T, typename U>
float dotProduct(const vector3<T>& a, const vector3<U>& b) {
    return a * b;
}

template<typename T>
float vectorMagnitude(const vector3<T>& a) {
    return a.magnitude();
}

template<typename T, typename U>
vector3<T> projectVector(const vector3<T>& a, const vector3<U>& b) {
    return b * (a * b / (b * b));
}
#endif