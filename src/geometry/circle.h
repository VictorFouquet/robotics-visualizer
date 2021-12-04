#pragma once

#include "vector3d.h"

namespace Geometry
{
    class Circle
    {
    public:
        Circle(float radius, Vector3d p);
        ~Circle() = default;
        
    private:
        float m_radius;
        Vector3d m_center;
    };
}
