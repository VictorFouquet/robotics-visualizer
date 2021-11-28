#include "frame.h"

void Frame::addCircle(int x, int y, float r) 
{
    Circle c = Circle(x, y, r);

    m_circles.push_back(c);
}

void Frame::addCircleBorder(int x, int y, float r, float width) 
{
    CircleBorder c = CircleBorder(x, y, r, width);

    m_circlesBorders.push_back(c);
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

void Frame::addMessage(std::string message, std::string fontName, int x, int y, int s) 
{
    MessageBox m(message, fontName, x, y, s);

    m_messages.push_back(m);
}

void Frame::addArc(int x, int y, float r, float theta, float phi, float width) 
{
    Arc arc(x, y, r, theta, phi, width);

    m_arcs.push_back(arc);
}
