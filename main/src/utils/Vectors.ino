#include "Vectors.h"

vector::operator+(vector& u){
    return (vector){x + u.x, y + u.y, z + u.z};
}

vector::operator-(vector& u){
    return (vector){x - u.x, y - u.y, z - u.z};
}

vector::operator*(vector& u){
    return x * u.x + y * u.y + z * u.z;
}

vector operator*(float& a){
    return (vector){x * a, y * a, z * a};
}

vector operator/(float& a){
    return (vector){x / a, y / a, z / a};   
}

vector::length(){
    return sqrt(this * this)
}

vector addVectors(vector& a, vector& b){
    return (vector){a.x + b.x, a.y + b.y, a.z + b.z};
}

vector subtractVectors(vector& a, vector&b){
    return (vector){a.x - b.x, a.y - b.y, a.z - b.z};
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
