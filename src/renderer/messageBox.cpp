#include "messageBox.h"

#include <SDL2/SDL_ttf.h>

MessageBox::MessageBox(std::string message, std::string fontName, int x, int y, int s)
{
    TTF_Font* font = TTF_OpenFont(fontName.c_str(), s);
    if(!font)
        printf("TTF_OpenFont: %s\n", TTF_GetError());

    m_surfaceMessage = TTF_RenderText_Solid(font, message.c_str(), {255,255,255}); 


    m_message.x = x;  //controls the rect's x coordinate 
    m_message.y = y; // controls the rect's y coordinte
    m_message.w = m_surfaceMessage->w; // controls the width of the rect
    m_message.h = m_surfaceMessage->h; // controls the height of the rect

    TTF_CloseFont(font);
}

void MessageBox::render(SDL_Renderer *renderer) 
{
    SDL_Texture* Message = SDL_CreateTextureFromSurface(renderer, m_surfaceMessage);

    SDL_RenderCopy(renderer, Message, NULL, &m_message);
    SDL_DestroyTexture(Message);
    // SDL_FreeSurface(m_surfaceMessage);
}
