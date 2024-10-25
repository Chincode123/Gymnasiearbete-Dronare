#include "Vector.h"

vector::operator+(vector& u){
    return (vector){this->x + u.x, this->y + u.y, this->z + u.z};
}

vector::operator-(vector& u){
    return (vector){this->x - u.x, this->y - u.y, this->z - u.z};
}

vector::operator*(vector& u){
    return this->x * u.x + this->y * u.y + this->z * u.z;
}

vector::length(){
    return sqrt(this * this)
}

float dotProduct(vector& a, vector& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

float vectorLength(vector& a) {
    return sqrt(a * a);
}

vector crossProduct(vector& a, vector& b, int orientation){
    switch (orientation){
        // right-handed
        case 0:
            return (vector){a.y * b.z - a.z * b.y,
                            a.z * b.x - a.x * b.z,
                            a.x * b.y - a.y * b.x};
            break;
        // left-handed
        case 1:
            return (vector){a.z * b.y - a.y * b.z,
                            a.x * b.z - a.z * b.x,
                            a.y * b.x - a.x * b.y};
            break;
        default:
            return nullptr;
    }
}