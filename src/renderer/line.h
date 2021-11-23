#pragma once

#include "primitive.h"
#include "color.h"


class Line : public Primitive
{
public:
    Line() = default;
    Line(int x1, int y1, int x2, int y2);
    virtual ~Line() override;

    virtual bool render(SDL_Renderer *renderer) override;

    virtual void setColor(int r, int g, int b, int a);
    
    void setPoints(int x1, int y1, int x2, int y2);
private:
    int x1, x2, y1, y2;
    Color m_color;
};
