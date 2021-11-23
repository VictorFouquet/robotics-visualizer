#pragma once

#include <SDL2/SDL.h>


class Primitive
{
public:
    Primitive();
    virtual ~Primitive();

    virtual SDL_Rect getBounds();
    virtual bool render(SDL_Renderer *renderer);
};
