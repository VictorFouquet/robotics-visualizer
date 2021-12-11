#pragma once
#include <string>
#include <SDL2/SDL.h>

class MessageBox
{
public:
    MessageBox() = default;
    MessageBox(std::string message, std::string fontName, int x, int y, int s);
    ~MessageBox() = default;

    void render(SDL_Renderer *renderer);
private:
    std::string m_fontName;
    SDL_Rect m_message;
    SDL_Surface* m_surfaceMessage;
};
