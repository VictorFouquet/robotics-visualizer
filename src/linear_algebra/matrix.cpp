#include "matrix.h"

#include <iostream>
#include <math.h>

#define PI 3.14159265

static float rad(float n)
{
    return 2 * PI * (n / 360);
}

Matrix::Matrix(float n) 
{
    for (int i = 0; i < 16; i++)
        values[i] = n; 
}

Matrix Matrix::operator* (Matrix m) 
{
    Vector3d col1 = Vector3d(values[0], values[4], values[8],  values[12]);
    Vector3d col2 = Vector3d(values[1], values[5], values[9],  values[13]);
    Vector3d col3 = Vector3d(values[2], values[6], values[10], values[14]);
    Vector3d col4 = Vector3d(values[3], values[7], values[11], values[15]);

    Vector3d mCol1 = Vector3d(m.values[0], m.values[4], m.values[8],  m.values[12]);
    Vector3d mCol2 = Vector3d(m.values[1], m.values[5], m.values[9],  m.values[13]);
    Vector3d mCol3 = Vector3d(m.values[2], m.values[6], m.values[10], m.values[14]);
    Vector3d mCol4 = Vector3d(m.values[3], m.values[7], m.values[11], m.values[15]);

    Vector3d rCol1 = col1 * mCol1.x + col2 * mCol1.y + col3 * mCol1.z + col4 * mCol1.w;
    Vector3d rCol2 = col1 * mCol2.x + col2 * mCol2.y + col3 * mCol2.z + col4 * mCol2.w;
    Vector3d rCol3 = col1 * mCol3.x + col2 * mCol3.y + col3 * mCol3.z + col4 * mCol3.w;
    Vector3d rCol4 = col1 * mCol4.x + col2 * mCol4.y + col3 * mCol4.z + col4 * mCol4.w;

    Matrix result = Matrix();
    result.values[0]  = rCol1.x;
    result.values[1]  = rCol2.x;
    result.values[2]  = rCol3.x;
    result.values[3]  = rCol4.x;

    result.values[4]  = rCol1.y;
    result.values[5]  = rCol2.y;
    result.values[6]  = rCol3.y;
    result.values[7]  = rCol4.y;

    result.values[8]  = rCol1.z;
    result.values[9]  = rCol2.z;
    result.values[10] = rCol3.z;
    result.values[11] = rCol4.z;

    result.values[12] = rCol1.w;
    result.values[13] = rCol2.w;
    result.values[14] = rCol3.w;
    result.values[15] = rCol4.w;

    return result;
}

Vector3d Matrix::operator* (Vector3d v) 
{
    float x = v.x * values[0]  + v.y * values[1]  + v.z * values[2]  + v.w * values[3];
    float y = v.x * values[4]  + v.y * values[5]  + v.z * values[6]  + v.w * values[7];
    float z = v.x * values[8]  + v.y * values[9]  + v.z * values[10] + v.w * values[11];
    float w = v.x * values[12] + v.y * values[13] + v.z * values[14] + v.w * values[15];

    Vector3d result = Vector3d(x, y, z, w);

    return result;
}

void Matrix::print() const
{
    for (int i = 0; i < 4; i++)
    {   
        std::cout << "{ ";
        for (int j = 0; j < 4; j++)
            std::cout << values[i * 4 + j] << (j == 3 ? " }\n" : ", ");
    }
    std::cout << std::endl;
}

Matrix Matrix::identity() 
{
    Matrix identity = Matrix();

    identity.values[0]  = 1.0f;
    identity.values[5]  = 1.0f;
    identity.values[10] = 1.0f;
    identity.values[15] = 1.0f;

    return identity;
}

Matrix Matrix::scale(float x, float y, float z) 
{
    Matrix scale = Matrix();

    scale.values[0]  = x;
    scale.values[5]  = y;
    scale.values[10] = z;
    scale.values[15] = 1.0f;

    return scale;
}

Matrix Matrix::translate(float x, float y, float z) 
{
    Matrix translate = Matrix();

    translate.values[0]  = 1.0f;
    translate.values[5]  = 1.0f;
    translate.values[10] = 1.0f;
    translate.values[15] = 1.0f;

    translate.values[3]  = x;
    translate.values[7]  = y;
    translate.values[11] = z;

    return translate;
}

Matrix Matrix::rotate(float x, float y, float z) 
{
    Matrix rotX;
    Matrix rotY;
    Matrix rotZ;

    if (x != 0.f)
    {
        rotX.values[0]  =  1.0f;
        rotX.values[5]  =  cos(rad(x));
        rotX.values[6]  = -sin(rad(x));
        rotX.values[9]  =  sin(rad(x));
        rotX.values[10] =  cos(rad(x));
        rotX.values[15] =  1.0f;
    }
    else
        rotX = Matrix::identity();


    if (y != 0.f)
    {
        rotY.values[0]  =  cos(rad(y));
        rotY.values[2]  =  sin(rad(y));
        rotY.values[5]  =  1.0f;
        rotY.values[8]  = -sin(rad(y));
        rotY.values[10] =  cos(rad(y));
        rotY.values[15] =  1.0f;
    }
    else
        rotY = Matrix::identity();


    if (z != 0.f)
    {
        rotZ.values[0]  =  cos(rad(z));
        rotZ.values[1]  = -sin(rad(z));
        rotZ.values[4]  =  sin(rad(z));
        rotZ.values[5]  =  cos(rad(z));
        rotZ.values[10] =  1.0f;
        rotZ.values[15] =  1.0f;
    } 
    else
        rotZ = Matrix::identity();

    Matrix rotate = rotX * rotY * rotZ;

    return rotate;
}

Matrix Matrix::orthographicProj(float left, float right, float top, float bottom, float near, float far) 
{
    // Space to render and canonical space can be represented by boxes, with a near and far planes.
    // Boxes can be defined by two points : bottom left corner of near plane and top right corner of far plane.
    
    // Step 1: translate center c of space box near plane to origin;

    Vector3d nearCenter = Vector3d((right + left) / 2, (top + bottom) / 2, near);
    Matrix translateOrigin = Matrix::translate(-nearCenter.x, -nearCenter.y, -nearCenter.z);
    
    // Step 2: Scale matrix to size of canonical;
    Matrix scaleBox = Matrix::scale(2 / (right - left), 2 / (top - bottom), 2 / (far - near));

    Matrix orthProj = scaleBox * translateOrigin;

    return orthProj;
}

Matrix Matrix::viewportTransform(float width, float height) 
{
    Matrix translation = Matrix::translate(width/2, height/2, 0.f);
    Matrix scaling = Matrix::scale(width/2, height/2, 1.f);

    Matrix transformation = translation * scaling;

    return transformation;
}

Vector3d Matrix::getColumn(int col) 
{
    return Vector3d(values[col], values[col + 4], values[col + 8]);
}

Vector3d Matrix::getRow(int row) 
{
    return Vector3d(values[row * 4], values[row * 4 + 1], values[row * 4 + 2]);
}