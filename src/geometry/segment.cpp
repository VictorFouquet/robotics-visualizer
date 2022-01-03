#include "segment.h"
namespace Geometry
{
    Segment::Segment(Vector3d p1, Vector3d p2) 
        : m_p1(p1), m_p2(p2), m_delta((p2 - p1))
    {
        
    }
    
    bool Segment::containsPoint(Vector3d p) 
    {
        return (
            m_p2.x <= std::max(m_p1.x, p.x) &&
            m_p2.x >= std::min(m_p1.x, p.x)
        ) && (
            m_p2.y <= std::max(m_p1.y, p.y) &&
            m_p2.y >= std::min(m_p1.y, p.y)
        );
    }
    
    bool Segment::intersectsWithSegment(Segment s) 
    {
        Vector3d p1 = m_p1, p2 = m_p2, q1 = s.getOrigin(), q2 = s.getEndPoint();

        int o1 = p1.orientation(p2, q1), o2 = p1.orientation(p2,q2);
        int o3 = q1.orientation(q2, p1), o4 = q1.orientation(q2,p2);
        
        Segment s1 = Segment(p1, q1);
        Segment s2 = Segment(p1, q2);
        Segment s3 = Segment(q1, p1);
        Segment s4 = Segment(q1, p2);

        return (o1 != o2 && o3 != o4) ||
            (o1 == 0 && s1.containsPoint(p2)) ||
            (o2 == 0 && s2.containsPoint(p2)) ||
            (o3 == 0 && s3.containsPoint(q2)) ||
            (o4 == 0 && s4.containsPoint(q2));
    }
    
    Vector3d Segment::projectOntoSegment(Segment s) 
    {
        float d = s.getDelta().dot(s.getDelta());

        if (d > 0.f) {
            float dp = m_delta.dot(s.getDelta());
            float coef = dp / d;

            return s.getDelta() * coef; 
        }
        
        return Vector3d();
    }
    
    Vector3d Segment::intersectsCircle(Circle c) 
    {
        if (
            m_p1.distanceToVector(c.getCenter()) < c.getRadius() ||
            m_p2.distanceToVector(c.getCenter()) < c.getRadius()
        ) return true;

        Segment s1 = Segment(m_p1, c.getCenter());
        float p = m_delta.unit().dot(s1.getDelta());
        Segment s2 = Segment(
            m_p1,
            (m_delta.unit()) * p + m_p1
        );
        Segment s3 = Segment(c.getCenter(), s2.getEndPoint());

        return (
            s3.magnitude() < c.getRadius() &&
            magnitude() >= s2.magnitude() &&
            m_delta.dot(s2.getDelta()) >= 0
        );
    }
}