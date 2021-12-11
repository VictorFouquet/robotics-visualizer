#include <math.h>
#include "arc.h"

#define PI 3.14159265

static float rad(float n)
{
    return 2 * PI * (n / 360);
}

Arc::Arc(int x, int y, float r, float theta, float phi, float width) 
    : m_x(x), m_y(y), m_r(r), m_min(rad(theta)), m_max(rad(phi)), m_width(width)
{
    m_color = { 50, 255, 50, 255 };

    m_rectangle.x = x - r;
    m_rectangle.y = y - r;
    m_rectangle.w = r * 2;
    m_rectangle.h = r * 2;
}

Arc::~Arc() 
{
    
}

int Arc::orientation(Vertex p, Vertex q, Vertex r)
{
    // Returns 0 if colinear, 1 if clockwise, -1 if counter clockwise
    long int a = (q.y - p.y) * (r.x - q.x);
    long int b = (q.x - p.x) * (r.y - q.y);
    long int ori = a - b;

    return ori == 0 ? ori : (ori > 0) ? 1 : -1;
}

SDL_Rect Arc::getBounds() 
{
    return m_rectangle;
}

bool Arc::render(SDL_Renderer *renderer) 
{
    SDL_SetRenderDrawColor(renderer, m_color.r, m_color.g, m_color.b, m_color.a);

    Vertex a = { (m_r - m_width) * std::cos(m_max) + m_x, (m_r - m_width) * std::sin(m_max) + m_y };
    Vertex b = { m_r * std::cos(m_max) + m_x, m_r * std::sin(m_max) + m_y };
    Vertex c = { (m_r - m_width) * std::cos(m_min) + m_x, (m_r - m_width) * std::sin(m_min) + m_y };
    Vertex d = { m_r * std::cos(m_min) + m_x, m_r * std::sin(m_min) + m_y };

    for (int y = m_rectangle.y; y < m_rectangle.y + m_rectangle.h; y ++) 
    {
        for (int x = m_rectangle.x; x < m_rectangle.x + m_rectangle.w; x ++)
        {
            Vertex p = { x, y };
            int deltaX = x - m_x;
            int deltaY = y - m_y;

            int or1 = orientation(a, b, p);
            int or2 = orientation(c, d, p);

            if
            (   
                or1 == -1 &&
                or2 == 1 &&
                std::sqrt(deltaX * deltaX + deltaY * deltaY) <= m_r && 
                std::sqrt(deltaX * deltaX + deltaY * deltaY) >= m_r - m_width
            )
                SDL_RenderDrawPoint(renderer, x, y);

        }
    }
    return true;
}

void Arc::setColor(int r, int g, int b, int a) 
{
    m_color = { r, g, b, a };
}

void Arc::setBounds(int x, int y, float r) 
{
    m_x = x, m_y = y, m_r = r;

    m_rectangle.x = x - r;
    m_rectangle.y = y - r;
    m_rectangle.w = r * 2;
    m_rectangle.h = r * 2;
}
