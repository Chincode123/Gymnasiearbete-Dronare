#ifndef VECTORS_H_
#define VECTORS_H_

// 3-Dimensional vector
struct vector {
    float x, y, z;

    float magnitude();
};

// Add two vectors
// return: u + v 
vector operator+(vector& u, vector& v);
vector operator+=(vector& u, vector& v);

// Subtract two vectors
// return: u - v 
vector operator-(vector& u, vector& v);
vector operator-=(vector& u, vector& v);

// Dot product of two vectors
// return: u *(dot) v 
float operator*(vector& u, vector& v);

// Multiplies a vector with a scalar
// return: (vector)u * (scalar)v 
vector operator*(vector& u, float a);
vector operator*=(vector& u, float a);

// Devides a vector with a scalar
// return: (vector)u / (scalar)a 
vector operator/(vector& u, float a);
vector operator/=(vector& u, float a);

// Add vectors: a and b
vector addVectors(vector& a, vector&b);

// Subtracts vector: b from vector: a
vector subtractVectors(vector& a, vector&b);

// Dot product of vectors: a and b
float dotProduct(vector &a, vector &b);

float vectorMagnitude(vector &a);

// Projects vector: a on vector: b
vector projectVector(vector& a, vector& b);

#endif