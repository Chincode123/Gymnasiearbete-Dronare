#ifndef VECTORS_H_
#define VECTORS_H_

// 3-Dimensional vector
template<typename T>
struct vector3 {
    T x, y, z;

    T magnitude();
};

// Add two vectors
// return: u + v 
vector3 operator+(const vector3& u, const vector3& v);
vector3 operator+=(vector3& u, const vector3& v);

// Subtract two vectors
// return: u - v 
vector3 operator-(const vector3& u, const vector3& v);
vector3 operator-=(vector3& u, const vector3& v);

// Dot product of two vectors
// return: u *(dot) v 
float operator*(const vector3& u, const vector3& v);

// Multiplies a vector with a scalar
// return: (vector)u * (scalar)v 
vector3 operator*(const vector3& u, float a);
vector3 operator*=(vector3& u, float a);

// Devides a vector with a scalar
// return: (vector)u / (scalar)a 
vector3 operator/(const vector3& u, float a);
vector3 operator/=(vector3& u, float a);

// Add vectors: a and b
vector3 addVectors(const vector3& a, const vector3& b);

// Subtracts vector: b from vector: a
vector3 subtractVectors(const vector3& a, const vector3&b);

// Dot product of vectors: a and b
float dotProduct(const vector3& a, const vector3& b);

float vectorMagnitude(const vector& a);

// Projects vector: a on vector: b
vector3 projectVector(const vector3& a, const vector3& b);

#endif