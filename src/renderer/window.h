#pragma once

#include <SDL2/SDL.h>
#include <vector>
#include "frame.h"

class Window
{
    public:
        Window() = default;
        Window(int width, int height);

        bool onInit();
        void pollEvents();
        void onEvent(SDL_Event *event);
        void onLoop();
        void onRender(Frame frame);
        void onExit();

        inline bool isOpened() const { return m_isOpened; }
        
    private:
        int m_width;
        int m_height; 

        bool m_isOpened;
        SDL_Window *m_window;
        SDL_Renderer *m_renderer;
};
