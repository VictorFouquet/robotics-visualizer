#pragma once

#include "primitive.h"
#include "color.h"


class Circle : public Primitive
{
public:
    Circle() = default;
    Circle(int x, int y, float r);
    virtual ~Circle() override;
    virtual SDL_Rect getBounds() override;
    virtual bool render(SDL_Renderer *renderer) override;

    virtual void setColor(int r, int g, int b, int a);
    
    void setBounds(int x, int y, float r);
private:
    int m_x, m_y;
    float m_r;
    Color m_color;
    SDL_Rect m_rectangle;
};
