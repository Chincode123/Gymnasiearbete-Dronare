#ifndef VECTORS_H_
#define VECTORS_H_

// 3-Dimensional vector
template<typename T>
struct vector3 {
    T x, y, z;

    float magnitude();

    // Assignment operator
    template<typename U>
    vector3<T>& operator=(const vector3<U>& other);
};

// Add two vectors
// return: u + v 
template<typename T, typename U>
vector3<T> operator+(const vector3<T>& u, const vector3<U>& v);
template<typename T, typename U>
vector3<T>& operator+=(vector3<T>& u, const vector3<U>& v);

// Subtract two vectors
// return: u - v 
template<typename T, typename U>
vector3<T> operator-(const vector3<T>& u, const vector3<U>& v);
template<typename T, typename U>
vector3<T>& operator-=(vector3<T>& u, const vector3<U>& v);

// Dot product of two vectors
// return: u *(dot) v 
template<typename T, typename U>
float operator*(const vector3<T>& u, const vector3<U>& v);

// Multiplies a vector with a scalar
// return: (vector)u * (scalar)v 
template<typename T>
vector3<T> operator*(const vector3<T>& u, float a);
template<typename T>
vector3<T>& operator*=(vector3<T>& u, float a);

// Divides a vector with a scalar
// return: (vector)u / (scalar)a 
template<typename T>
vector3<T> operator/(const vector3<T>& u, float a);
template<typename T>
vector3<T>& operator/=(vector3<T>& u, float a);

// Add vectors: a and b
template<typename T, typename U>
vector3<T> addVectors(const vector3<T>& a, const vector3<U>& b);

// Subtracts vector: b from vector: a
template<typename T, typename U>
vector3<T> subtractVectors(const vector3<T>& a, const vector3<U>& b);

// Dot product of vectors: a and b
template<typename T, typename U>
float dotProduct(const vector3<T>& a, const vector3<U>& b);

template<typename T>
float vectorMagnitude(const vector3<T>& a);

// Projects vector: a on vector: b
template<typename T, typename U>
vector3<T> projectVector(const vector3<T>& a, const vector3<U>& b);

#endif