#pragma once

#pragma once

#include "primitive.h"
#include "color.h"


class CircleBorder : public Primitive
{
public:
    CircleBorder() = default;
    CircleBorder(int x, int y, float r, float width);
    virtual ~CircleBorder() override;
    virtual SDL_Rect getBounds() override;
    virtual bool render(SDL_Renderer *renderer) override;

    virtual void setColor(int r, int g, int b, int a);
    
    void setBounds(int x, int y, float r);
private:
    int m_x, m_y;
    float m_r, m_width;
    Color m_color;
    SDL_Rect m_rectangle;
};
