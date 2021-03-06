#pragma once

#include "arc.h"
#include "circle.h"
#include "circleBorder.h"
#include "color.h"
#include "line.h"
#include "messageBox.h"
#include "polygon.h"
#include "rectangle.h"
#include "triangle.h"

#include <vector>


class Frame
{
public:
    Frame() = default;
    ~Frame() = default;

    void addArc(int x, int y, float r, float theta, float phi, float width);
    void addCircle(int x, int y, float r);
    void addCircle(int x, int y, float radius, float r, float g, float b, float a=255.f);
    void addCircleBorder(int x, int y, float r, float width);
    void addLine(int p1x, int p1y, int p2x, int p2y);
    void addPolygon();
    void addRectangle(int width, int height, int x, int y, int r, int g, int b);
    void addTriangle();
    void addMessage(std::string message, std::string fontName, int x, int y, int s);

    std::vector<Arc> getArcs() const { return m_arcs; }
    std::vector<Circle> getCircles() const { return m_circles; }
    std::vector<CircleBorder> getCirclesBorder() const { return m_circlesBorders; }
    std::vector<Line> getLines() const { return m_lines; }
    std::vector<Polygon> getPolygons() const { return m_polygons; }
    std::vector<Rectangle> getRectangles() const { return m_rectangles; }
    std::vector<Triangle> getTriangles() const { return m_triangles; }
    std::vector<MessageBox> getMessages() const { return m_messages; }

private:
    std::vector<Arc>          m_arcs;
    std::vector<Circle>       m_circles;
    std::vector<CircleBorder> m_circlesBorders;
    std::vector<Line>         m_lines;
    std::vector<Polygon>      m_polygons;
    std::vector<Rectangle>    m_rectangles;
    std::vector<Triangle>     m_triangles;
    std::vector<MessageBox>   m_messages;
};
