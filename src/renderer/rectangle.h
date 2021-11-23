#pragma once

#include "primitive.h"
#include "color.h"

class Rectangle : public virtual Primitive
{
public:
    Rectangle() = default;
    Rectangle(int x, int y, int w, int h);
    virtual ~Rectangle() override;
    virtual SDL_Rect getBounds() override;
    virtual bool render(SDL_Renderer *renderer) override;

    virtual void setColor(int r, int g, int b, int a);
    
    void setBounds(int x, int y, int w, int h);
private:
    SDL_Rect m_rectangle;
    Color m_color;
};
