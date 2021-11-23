#include "polygon.h"
#include <algorithm>

Polygon::Polygon(std::vector<Vertex> points)
{
    setBounds(points);
    setColor(255, 255, 255, 255);
    
}

Polygon::~Polygon() 
{
    
}

SDL_Rect Polygon::getBounds() 
{
    return m_rectangle;
}

int Polygon::orientation(Vertex p, Vertex q, Vertex r)
{
    // Returns 0 if colinear, 1 if clockwise, -1 if counter clockwise
    long int a = (q.y - p.y) * (r.x - q.x);
    long int b = (q.x - p.x) * (r.y - q.y);
    long int ori = a - b;

    return ori == 0 ? ori : (ori > 0) ? 1 : -1;
}

bool Polygon::onSegment(Vertex p, Vertex q, Vertex r)
{
    if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
        q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y)
    )
        return true;
    return false;
}

bool Polygon::intersect(Vertex p1, Vertex q1, Vertex p2, Vertex q2)
{
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    if (o1 != o2 && o3 != o4)
        return true;

    if (o1 == 0 && onSegment(p1, p2, q1))
        return true;
    if (o2 == 0 && onSegment(p1, q2, q1))
        return true;
    if (o3 == 0 && onSegment(p2, p1, q2))
        return true;
    if (o4 == 0 && onSegment(p2, q1, q2))
        return true;
    
    return false;
}

bool Polygon::render(SDL_Renderer *renderer) 
{
    SDL_SetRenderDrawColor(renderer, m_color.r, m_color.g, m_color.b, m_color.a);
    int n = m_points.size();

    for (int x = 1; x < m_rectangle.w - 1; x++)
    {
        for (int y = 1; y < m_rectangle.h - 1; y++)
        {
            int count = 0;
            for (int i = 0; i < n; i++)
            {
                Vertex p1 = { x + m_rectangle.x, y + m_rectangle.y };
                Vertex p2 = { 10000, p1.y + 1 };

                Vertex q1 = m_points[i];
                Vertex q2 = m_points[(i + 1) % n];

                if (intersect(p1, p2, q1, q2))
                {
                    count ++;
                }
            }

            if (count & 1)
                SDL_RenderDrawPoint(renderer, x + m_rectangle.x, y + m_rectangle.y);
        }
    }
    return true;
}

void Polygon::setColor(int r, int g, int b, int a) 
{
    m_color = { r, g, b, a };
}

void Polygon::setBounds(std::vector<Vertex> points) 
{
    int minX = 10000;
    int minY = 10000;
    int maxX = 0;
    int maxY = 0;

    for (Vertex point : points)
    {
        Vertex v = { point.x, point.y };
        m_points.push_back(v);

        if (minX > point.x)
            minX = point.x;
        if (maxX < point.x)
            maxX = point.x;
        if (minY > point.y)
            minY = point.y;
        if (maxY < point.y)
            maxY = point.y;
    }

    m_rectangle.x = minX, m_rectangle.w = maxX - minX;
    m_rectangle.y = minY, m_rectangle.h = maxY - minY;
}
