#pragma once

#include <iostream>
#include <math.h>

class Vector3d
{
public:
    Vector3d(float x, float y, float z, float w = 1.0f)
        : x(x), y(y), z(z), w(w) {}
    
    Vector3d(float x, float y)
        : x(x), y(y), z(0.f), w(1.f) {}

    Vector3d(float n)
        : x(n), y(n), z(n), w(1.f) {}
    
    Vector3d()
        : x(0.0f), y(0.0f), z(0.0f), w(1.f) {}
    
    inline Vector3d operator+(Vector3d v) const { return Vector3d(x + v.x, y + v.y, z + v.z, w + v.w); }
    inline Vector3d operator-(Vector3d v) const { return Vector3d(x - v.x, y - v.y, z - v.z, w - v.w); }
    inline Vector3d operator*(float n) const { return Vector3d(x * n, y * n, z * n, w * n) ;}
    inline Vector3d operator/(float n) const { return Vector3d(x / n, y / n, z / n, w / n) ;}

    inline float magnitude() const { return std::sqrt(x * x + y * y + z * z); }
    inline float dot(Vector3d vec) const { return x * vec.x + y * vec.y + z * vec.z; }
    
    inline float distanceToVector(Vector3d v) { return std::sqrt(x * v.x + y * v.y + z * v.z ); }
    inline float angleToVector(Vector3d v) { float x = dot(v) / (magnitude() * v.magnitude()); return x > 1.f ? 1.f : x < -1.f ? -1.f : x; }

    inline void print() const { std::cout << "{ " << x << ", " << y << ", " << z << ", " << w << " }" << std::endl; }
public:
    float x, y, z, w;
};