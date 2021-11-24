#pragma once

#include <vector>

#include "primitive.h"
#include "vertex.h"
#include "color.h"


class Triangle : public Primitive
{
public:
    Triangle() = default;
    Triangle(Vertex a, Vertex b, Vertex c);
    virtual ~Triangle() override;
    virtual SDL_Rect getBounds() override;
    virtual bool render(SDL_Renderer *renderer) override;

    virtual void setColor(int r, int g, int b, int a);
    
    void setBounds(Vertex a, Vertex b, Vertex c);

public:
    static bool sortVertices(Vertex v1, Vertex v2);
    static void interpolate(Vertex v1, Vertex v2, std::vector<Vertex>* outline);
private:
    Vertex m_pointA, m_pointB, m_pointC;
    SDL_Rect m_rectangle;
    Color m_color;
};
