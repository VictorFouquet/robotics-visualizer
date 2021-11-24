#include "frame.h"

void Frame::addCircle(int x, int y, float r) 
{
    Circle c = Circle(x, y, r);

    m_circles.push_back(c);
}

void Frame::addLine(int x1, int y1, int x2, int y2) 
{
    Line line(x1, y1, x2, y2);

    m_lines.push_back(line);
}

void Frame::addPolygon() 
{
    
}

void Frame::addRectangle() 
{
    
}

void Frame::addTriangle() 
{
    
}
