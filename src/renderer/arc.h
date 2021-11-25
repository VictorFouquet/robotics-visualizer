#pragma once

#include "primitive.h"
#include "color.h"
#include "vertex.h"

class Arc : public Primitive
{
public:
    Arc() = default;
    Arc(int x, int y, float r, float theta, float phi, float width);
    virtual ~Arc() override;
    virtual SDL_Rect getBounds() override;
    virtual bool render(SDL_Renderer *renderer) override;
    virtual void setColor(int r, int g, int b, int a);
    
    int orientation(Vertex p, Vertex q, Vertex r);
    void setBounds(int x, int y, float r);
private:
    int m_x, m_y;
    float m_r, m_min, m_max, m_width;
    Color m_color;
    SDL_Rect m_rectangle;
};