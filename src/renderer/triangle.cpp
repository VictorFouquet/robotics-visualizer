#include <algorithm>
#include <stdlib.h>
#include <iostream>

#include "triangle.h"


Triangle::Triangle(Vertex a, Vertex b, Vertex c) 
    :m_pointA(a), m_pointB(b), m_pointC(c)
{
    m_rectangle.x = std::min(std::min(a.x, b.x), c.x);
    m_rectangle.w = std::max(std::max(a.x, b.x), c.x) - m_rectangle.x;
    m_rectangle.y = std::min(std::min(a.y, b.y), c.y);
    m_rectangle.h = std::max(std::max(a.y, b.y), c.y) - m_rectangle.y;

    m_color = { 255, 255, 255, 255 };
}

Triangle::~Triangle() 
{

}

SDL_Rect Triangle::getBounds() 
{
    return m_rectangle;
}

bool Triangle::sortVertices(Vertex v1, Vertex v2)
{
    if (v1.y == v2.y)
    {
        return v1.x < v2.x;
    }
    return v1.y < v2.y;
}

void Triangle::interpolate(Vertex v1, Vertex v2, std::vector<Vertex>* outline) 
{
    int x1 = v1.x, x2 = v2.x, y1 = v1.y, y2 = v2.y;

    if (y1 != y2)
    {
        if (y2 < y1)
        {
            std::swap(x1, x2);
            std::swap(y1, y2);
        }
        int deltaX = x2 - x1, deltaY = y2 -y1;
        float stepX = (float)deltaX / (float)deltaY;

        for (int i = 0; i < deltaY; i++)
        {
            int dx = int(x1 + stepX * i);
            Vertex v = { dx, y1 + i };
            outline->push_back(v);
        }
    }
}

bool Triangle::render(SDL_Renderer *renderer) 
{
    SDL_SetRenderDrawColor(renderer, m_color.r, m_color.g, m_color.b, m_color.a);

    std::vector<Vertex> outline;
    interpolate(m_pointA, m_pointB, &outline);
    interpolate(m_pointB, m_pointC, &outline);
    interpolate(m_pointC, m_pointA, &outline);

    std::sort(outline.begin(), outline.end(), sortVertices);

    for (int i = 0; i < outline.size(); i += 2)
    {
        int x1 = outline[i].x, x2 = outline[i+1].x, y = outline[i].y;
        if (x1 > x2)
            std::swap(x1, x2);
        
        for (int x = x1; x < x2; x++)
            SDL_RenderDrawPoint(renderer, x, y);
    }

    return true;
}

void Triangle::setColor(int r, int g, int b, int a) 
{
    m_color = { r, g, b, a };
}

void Triangle::setBounds(Vertex a, Vertex b, Vertex c) 
{
    m_pointA = a, m_pointB = b, m_pointC = c;

    m_rectangle.x = std::min(std::min(a.x, b.x), c.x);
    m_rectangle.w = std::max(std::max(a.x, b.x), c.x) - m_rectangle.x;
    m_rectangle.y = std::min(std::min(a.y, b.y), c.y);
    m_rectangle.h = std::max(std::max(a.y, b.y), c.y) - m_rectangle.y;
}
