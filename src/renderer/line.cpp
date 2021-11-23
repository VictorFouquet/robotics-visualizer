#include "line.h"

Line::Line(int x1, int y1, int x2, int y2) 
    :x1(x1), y1(y1), x2(x2), y2(y2)
{
    m_color = { 255, 255, 255, 255 };
}

Line::~Line() 
{
    
}

bool Line::render(SDL_Renderer *renderer) 
{
    SDL_SetRenderDrawColor(renderer, m_color.r, m_color.g, m_color.b, m_color.a);
    SDL_RenderDrawLine(renderer, x1, y1, x2, y2);

    return true;
}

void Line::setColor(int r, int g, int b, int a) 
{
    m_color = { r, g, b, a };
}

void Line::setPoints(int x1, int y1, int x2, int y2) 
{
    x1 = x1, x2 = x2, y1 = y1, y2 = y2;
}
