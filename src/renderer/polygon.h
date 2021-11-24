#pragma once

#include <vector>

#include "primitive.h"
#include "color.h"
#include "vertex.h"


class Polygon : public Primitive
{
public:
    Polygon() = default;
    Polygon(std::vector<Vertex> points);
    virtual ~Polygon() override;
    virtual SDL_Rect getBounds() override;
    virtual bool render(SDL_Renderer *renderer) override;

    virtual void setColor(int r, int g, int b, int a);
    
    void setBounds(std::vector<Vertex> points);

public:
    static int orientation(Vertex p1, Vertex p2, Vertex p3);
    static bool onSegment(Vertex p, Vertex q, Vertex r);
    static bool intersect(Vertex p1, Vertex q1, Vertex p2, Vertex q2);

private:
    SDL_Rect m_rectangle;
    std::vector<Vertex> m_points;
    Color m_color;
};
