#ifndef VECTORS_H_
#define VECTORS_H_

// 3-Dimensional vector
struct vector {
    float x, y, z;

    float magnitude();
};

// Add two vectors
// return: u + v 
vector operator+(const vector& u, const vector& v);
vector operator+=(vector& u, const vector& v);

// Subtract two vectors
// return: u - v 
vector operator-(const vector& u, const vector& v);
vector operator-=(vector& u, const vector& v);

// Dot product of two vectors
// return: u *(dot) v 
float operator*(const vector& u, const vector& v);

// Multiplies a vector with a scalar
// return: (vector)u * (scalar)v 
vector operator*(const vector& u, float a);
vector operator*=(vector& u, float a);

// Devides a vector with a scalar
// return: (vector)u / (scalar)a 
vector operator/(const vector& u, float a);
vector operator/=(vector& u, float a);

// Add vectors: a and b
vector addVectors(const vector& a, const vector& b);

// Subtracts vector: b from vector: a
vector subtractVectors(const vector& a, const vector&b);

// Dot product of vectors: a and b
float dotProduct(const vector& a, const vector& b);

float vectorMagnitude(const vector& a);

// Projects vector: a on vector: b
vector projectVector(const vector& a, const vector& b);

#endif