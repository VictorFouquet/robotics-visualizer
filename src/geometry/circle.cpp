#include "circle.h"
#include <stdlib.h>

namespace Geometry
{
    Circle::Circle(float radius, Vector3d p)
        : m_radius(radius), m_center(p) { }
        
    std::vector<Vector3d, Vector3d> Circle::getIntersectionPointsWithCircle(Circle c) 
    {
        // Ressource and proof can be found here: http://paulbourke.net/geometry/circlesphere/

        // First calculate the distance d between the center of the circles. d = ||P1 - P0||
        float d = m_center.distanceToVector(c.getCenter());
        if (
        // If d > r0 + r1 then there are no solutions, the circles are separate.
            ( d > (m_radius + c.getRadius()) ) ||
        // If d < |r0 - r1| then there are no solutions because one circle is contained within the other.
            ( d < std::abs(m_radius - c.getRadius()) ) ||
        // If d = 0 and r0 = r1 then the circles are coincident and there are an infinite number of solutions.
            ( (d == 0) && (m_radius == c.getRadius()) ) 
        ) return { Vector3d(), Vector3d() };
        
        /**
        Considering the two triangles P0P2P3 and P1P2P3 we can write

            a² + h² = r0² and b² + h² = r1²

        Using d = a + b we can solve for a,

            a = (r0² - r1² + d² ) / (2 d) 
        */
        float a = ((m_radius * m_radius) - (c.getRadius() * c.getRadius()) + (d * d)) / (d * 2);
        // Solve for h by substituting a into the first equation, h² = r0² - a²
        float h = std::sqrt((m_radius * m_radius) - (a * a));

        // So P2 = P0 + a ( P1 - P0 ) / d
        Vector3d p2 = m_center + ( (c.getCenter() - m_center) * a / d );
        /*
        And finally, P3 = (x3,y3) in terms of P0 = (x0,y0), P1 = (x1,y1) and P2 = (x2,y2), is

            x3 = x2 +- h ( y1 - y0 ) / d
            
            y3 = y2 -+ h ( x1 - x0 ) / d
        */
        float x3 = p2.x + ( h * (c.getCenter().y - m_center.y) / d );
        float x4 = p2.x - ( h * (c.getCenter().y - m_center.y) / d );
        float y3 = p2.y - ( h * (c.getCenter().x - m_center.x) / d );
        float y4 = p2.y + ( h * (c.getCenter().x - m_center.x) / d );

        return { Vector3d(x3, y3), Vector3d(x4,y4) };
    }
}