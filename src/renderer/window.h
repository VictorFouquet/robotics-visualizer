#pragma once

#include <SDL2/SDL.h>

class Window
{
    public:
        Window(int width, int height);

        int onExecute();
        bool onInit();
        void onEvent(SDL_Event *event);
        void onLoop();
        void onRender();
        void onExit();

    private:
        int m_width;
        int m_height; 

        bool m_isOpened;
        SDL_Window *m_window;
        SDL_Renderer *m_renderer;
};
