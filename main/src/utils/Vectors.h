#ifndef VECTORS_H_
#define VECTORS_H_

struct vector {
    float x, y, z;

    vector operator+(vector& u);
    vector operator-(vector& u);
    vector operator*(vector& u);

    vector operator*(float& a);
    vector operator/(float& a);

    float length();
};


// adds a and b together
vector addVectors(vector& a, vector&b);

// subtrackts b from a
vector subtractVectors(vector& a, vector&b);

// dotproduct of vectors a and b
float dotProduct(vector &a, vector &b);

float vectorLength(vector &a);

// projects vector a on to vector b
vector projectVector(vector& a, vector& b);

#endif