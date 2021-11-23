#include "rectangle.h"
#include <iostream>

Rectangle::Rectangle(int x, int y, int w, int h) 
{
    m_rectangle.x = x;
    m_rectangle.y = y;
    m_rectangle.w = w;    
    m_rectangle.h = h;

    m_color = { 255, 255, 255, 255 };
}

Rectangle::~Rectangle() 
{
    
}

SDL_Rect Rectangle::getBounds() 
{
    return m_rectangle;
}

void Rectangle::setBounds(int x, int y, int w, int h)
{
    m_rectangle.x = x;
    m_rectangle.y = y;
    m_rectangle.w = w;    
    m_rectangle.h = h;
}

bool Rectangle::render(SDL_Renderer *renderer) 
{
    SDL_SetRenderDrawColor(renderer, m_color.r, m_color.g, m_color.b, m_color.a);

    for (int y = m_rectangle.y; y < m_rectangle.y + m_rectangle.h; y ++) 
    {
        for (int x = m_rectangle.x; x < m_rectangle.x + m_rectangle.w; x ++)
        {
            SDL_RenderDrawPoint(renderer, x, y);
        }
    }
    return true;
}

void Rectangle::setColor(int r, int g, int b, int a) 
{
    m_color = { r, g, b, a };
}
