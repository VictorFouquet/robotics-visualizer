#pragma once

#include "vector3d.h"
#include <vector>

namespace Geometry
{
    class Circle
    {
    public:
        Circle(float radius, Vector3d p);
        ~Circle() = default;

        Vector3d getCenter() { return m_center; }
        float getRadius() { return m_radius; }

        void getIntersectionPointsWithCircle(Circle c, Vector3d& p1, Vector3d& p2);
        void getIntersectionPointsWithSegment(Vector3d a, Vector3d b, Vector3d& p1, Vector3d& p2);
    private:
        float m_radius;
        Vector3d m_center;
    };
}
