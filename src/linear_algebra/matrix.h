#pragma once

#include "vector3d.h"

class Matrix
{
public: 
    Matrix(float n = 0.0f);
    ~Matrix() = default;

    Vector3d operator* (Vector3d v);
    Matrix operator* (Matrix m);

    void print() const;
    
    static Matrix identity();
    static Matrix scale(float x, float y, float z);
    static Matrix translate(float x, float y, float z);
    static Matrix rotate(float x, float y, float z);

    static Matrix orthographicProj(float left, float right, float top, float bottom, float near, float far);
    static Matrix viewportTransform(float width, float height);

    Vector3d getColumn(int col);
    Vector3d getRow(int row);
public:
    float values[16];
};