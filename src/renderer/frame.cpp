#include "frame.h"

void Frame::addCircle(int x, int y, float r) 
{
    Circle c = Circle(x, y, r);

    m_circles.push_back(c);
}

void Frame::addCircle(int x, int y, float radius, float r, float g, float b, float a) 
{
    Circle c = Circle(x, y, radius);
    c.setColor(r, g, b, 255);
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

void Frame::addRectangle(int width, int height, int x, int y, int r, int g, int b) 
{
    Rectangle rectangle(x, y, width, height);
    rectangle.setColor(r, g, b, 255);
    m_rectangles.push_back(rectangle);
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
