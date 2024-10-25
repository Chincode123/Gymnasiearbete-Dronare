#ifndef VECTOR_H_
#define VECTOR_H_

struct vector {
    float x, y, z;

    vector operator+(vector& u);
    vector operator-(vector& u);
    vector operator*(vector& u);

    float length();
};

// dotproduct of vectors a and b
float dotProduct(vector &a, vector &b);

float vectorLength(vector &a);

// returns the vector c; the cross product of vectors a and b. orientainon: 0 - right-handed, 1 - left-handed
vector crossProduct(vector &a, vector &b, int orientation);

void test(){
    vector a = {1, 2, 3};
    vector b = {3, 2, 1};
}

#endif