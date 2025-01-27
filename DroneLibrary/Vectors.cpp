#include "Vectors.h"

float vector::magnitude(){
    return sqrt((*this) * (*this));
}

vector operator+(const vector& u, const vector& v) {
    return {u.x + v.x, u.y + v.y, u.z + v.z};
}

vector operator+=(vector& u, const vector& v) {
    u = u + v;
    return u;
}

vector operator-(const vector& u, const vector& v) {
    return {u.x - v.x, u.y - v.y, u.z - v.z};
}

vector operator-=(vector& u, const vector& v) {
    u = u - v;
    return u;
}

float operator*(const vector& u, const vector& v) {
    return u.x * v.x + u.y * v.y + u.z * v.z;
}

vector operator*(const vector& u, float a) {
    return {u.x * a, u.y * a, u.z * a};
}

vector operator*=(vector& u, float a) {
    u = u * a;
    return u;
}

vector operator/(const vector& u, float a) {
    if (a == 0)
        return {0, 0, 0};
    return {u.x / a, u.y / a, u.z / a};   
}

vector operator/=(vector& u, float a) {
    if (a == 0)
        return {0, 0, 0};
    u = u / a;
    return u;
}

vector addVectors(const vector& a, const vector& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

vector subtractVectors(const vector& a, const vector&b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

float dotProduct(const vector& a, const vector& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

float vectorMagnitude(const vector& a) {
    return sqrt(a * a);
}

vector projectVector(const vector& a, const vector& b) {
    return b * ((a * b) / (b * b));
}