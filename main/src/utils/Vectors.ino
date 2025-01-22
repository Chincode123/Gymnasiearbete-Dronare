#include "Vectors.h"

float vector::length(){
    return sqrt((*this) * (*this));
}

vector operator+(vector& u, vector& v) {
    return vector {u.x + v.x, u.y + v.y, u.z + v.z};
}

vector operator-(vector& u, vector& v) {
    return vector {u.x - v.x, u.y - v.y, u.z - v.z};
}

float operator*(vector& u, vector& v) {
    return u.x * v.x + u.y * v.y + u.z * v.z;
}

vector operator*(vector& u, float a) {
    return vector {u.x * a, u.y * a, u.z * a};
}

vector operator*(float a, vector& u) {
    return vector {u.x * a, u.y * a, u.z * a};
}

vector operator/(vector& u, float a) {
    return vector {u.x / a, u.y / a, u.z / a};   
}

vector addVectors(vector& a, vector& b){
    return vector {a.x + b.x, a.y + b.y, a.z + b.z};
}

vector subtractVectors(vector& a, vector&b){
    return vector {a.x - b.x, a.y - b.y, a.z - b.z};
}

float dotProduct(vector& a, vector& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

float vectorLength(vector& a) {
    return sqrt(a * a);
}

vector projectVector(vector& a, vector& b){
    return b * ((a * b) / (b * b));
}
