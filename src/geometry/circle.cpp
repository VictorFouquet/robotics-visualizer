#include "circle.h"
#include "vector3d.h"
#include <stdlib.h>
#include <vector>

namespace Geometry
{
    Circle::Circle(float radius, Vector3d p)
        : m_radius(radius), m_center(p) { }
        
    void Circle::getIntersectionPointsWithCircle(Circle c, Vector3d& ip1, Vector3d& ip2)
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
        ) return;
        
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
        ip1.x = p2.x + ( h * (c.getCenter().y - m_center.y) / d );
        ip2.x = p2.x - ( h * (c.getCenter().y - m_center.y) / d );
        ip1.y = p2.y - ( h * (c.getCenter().x - m_center.x) / d );
        ip2.y = p2.y + ( h * (c.getCenter().x - m_center.x) / d );
    }
    
    void Circle::getIntersectionPointsWithSegment(Vector3d va, Vector3d vb, Vector3d& p1, Vector3d& p2) 
    {
        // Ressource and proof can be founb here : https://newbedev.com/get-location-of-vector-circle-intersection
        
        // Circle equation = (x - h)² + (y - k)² = r²
        // Center = (h , k), r = radius
        float h = m_center.x, k = m_center.y, r = m_radius;
        // Vector has starting point (ax, ay) and ending point (bx, by)
        float ax = va.x, ay = va.y, bx = vb.x, by = vb.y;
        // Parametric equation for point on vector :
        // x(t) = (bx - ax)t + ax
        // y(t) = (by - ay)t + ay
        // Substitute x and y in circle equation by parametric equations :
        // ((bx - ax)t + ax - h)² + ((by - ay)t + ay - k)² = r²
        // Collect terms in form at² + bt + c = 0
        // Where:
        // a = (bx - ax)² + (by - ay)²
        float a = std::pow((bx - ax), 2) + std::pow((by - ay), 2);
        // b = 2(bx - ax)(ax - h) + 2(by - ay)(ay - k)
        float b = 2 * (bx - ax) * (ax - h) + 2 * (by - ay) * (ay - k);
        // c = (ax - h)² + (ay - k)² - r²
        float c = std::pow((ax - h), 2) + std::pow((ay - k), 2) - std::pow(r, 2);
        // Then t1 = (-b + sqrt(b² - 4ac)) / 2a
        float t1 = (-b + std::sqrt(std::pow(b, 2) - 4 * a * c)) / (2 * a);
        // And  t2 = (-b - sqrt(b² - 4ac)) / 2a
        float t2 = (-b - std::sqrt(std::pow(b, 2) - 4 * a * c)) / (2 * a);
        // Finally using parametric equations intersections points are (x(t1), y(t1)), (x(t2), y(t2))
        p1.x = (bx - ax) * t1 + ax;
        p1.y = (by - ay) * t1 + ay;
        p2.x = (bx - ax) * t2 + ax;
        p2.y = (by - ay) * t2 + ay;
    }
}