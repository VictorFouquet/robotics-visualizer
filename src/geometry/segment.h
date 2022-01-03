#pragma once

#include <vector>

#include "circle.h"
#include "vector3d.h"


namespace Geometry
{
    class Segment
    {
    public:
        Segment(Vector3d p1, Vector3d p2);
        ~Segment() = default;

        Vector3d getOrigin() { return m_p1; }
        Vector3d getEndPoint() { return m_p2; }
        Vector3d getDelta() { return m_delta; }

        Vector3d center() { return m_p1 + (m_delta / 2); }
        float magnitude() { return m_delta.magnitude(); }

        bool containsPoint(Vector3d p);
        bool intersectsWithSegment(Segment s);
        Vector3d projectOntoSegment(Segment s);
        Vector3d intersectsCircle(Circle c);
    public:
    
    private:
        Vector3d m_p1, m_p2;
        Vector3d m_delta;
    };
}
