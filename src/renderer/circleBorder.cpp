#include <math.h>
#include "circleBorder.h"

CircleBorder::CircleBorder(int x, int y, float r, float width) 
    : m_x(x), m_y(y), m_r(r), m_width(width)
{
    m_color = { 50, 150, 50, 255 };

    m_rectangle.x = x - r;
    m_rectangle.y = y - r;
    m_rectangle.w = r * 2;
    m_rectangle.h = r * 2;
}

CircleBorder::~CircleBorder() 
{
    
}

SDL_Rect CircleBorder::getBounds() 
{
    return m_rectangle;
}

bool CircleBorder::render(SDL_Renderer *renderer) 
{
    SDL_SetRenderDrawColor(renderer, m_color.r, m_color.g, m_color.b, m_color.a);

    for (int y = m_rectangle.y; y < m_rectangle.y + m_rectangle.h; y ++) 
    {
        for (int x = m_rectangle.x; x < m_rectangle.x + m_rectangle.w; x ++)
        {
            int deltaX = x - m_x;
            int deltaY = y - m_y;
            if 
            (
                std::sqrt(deltaX * deltaX + deltaY * deltaY) <= m_r &&
                std::sqrt(deltaX * deltaX + deltaY * deltaY) >= (m_r - m_width)
            )
                SDL_RenderDrawPoint(renderer, x, y);
        }
    }
    return true;
}

void CircleBorder::setColor(int r, int g, int b, int a) 
{
    m_color = { r, g, b, a };
}

void CircleBorder::setBounds(int x, int y, float r) 
{
    m_x = x, m_y = y, m_r = r;

    m_rectangle.x = x - r;
    m_rectangle.y = y - r;
    m_rectangle.w = r * 2;
    m_rectangle.h = r * 2;
}
