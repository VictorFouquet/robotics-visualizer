#include <math.h>
#include "circle.h"

Circle::Circle(int x, int y, float r) 
    : m_x(x), m_y(y), m_r(r)
{
    m_color = { 255, 255, 255, 255 };

    m_rectangle.x = x - r;
    m_rectangle.y = y - r;
    m_rectangle.w = r * 2;
    m_rectangle.h = r * 2;
}

Circle::~Circle() 
{
    
}

SDL_Rect Circle::getBounds() 
{
    return m_rectangle;
}

bool Circle::render(SDL_Renderer *renderer) 
{
    SDL_SetRenderDrawColor(renderer, m_color.r, m_color.g, m_color.b, m_color.a);

    for (int y = m_rectangle.y; y < m_rectangle.y + m_rectangle.h; y ++) 
    {
        for (int x = m_rectangle.x; x < m_rectangle.x + m_rectangle.w; x ++)
        {
            int deltaX = x - m_x;
            int deltaY = y - m_y;
            if (std::sqrt(deltaX * deltaX + deltaY * deltaY) <= m_r)
                SDL_RenderDrawPoint(renderer, x, y);
        }
    }
    return true;
}

void Circle::setColor(int r, int g, int b, int a) 
{
    m_color = { r, g, b, a };
}

void Circle::setBounds(int x, int y, float r) 
{
    m_x = x, m_y = y, m_r = r;

    m_rectangle.x = x - r;
    m_rectangle.y = y - r;
    m_rectangle.w = r * 2;
    m_rectangle.h = r * 2;
}
