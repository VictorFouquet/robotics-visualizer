#pragma once

#include "vector3d.h"

namespace Geometry
{
    class Circle
    {
    public:
        Circle(float radius, Vector3d p);
        ~Circle() = default;

        Vector3d getCenter() { return m_center; }
        float getRadius() { return m_radius; }
    private:
        float m_radius;
        Vector3d m_center;
    };
}
