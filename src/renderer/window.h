#pragma once

#include <SDL2/SDL.h>
#include <vector>
#include "primitive.h"
#include "circle.h"

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

        void drawCircle(int x, int y, int radius, int r, int g, int b);
        
    private:
        int m_width;
        int m_height; 

        bool m_isOpened;
        SDL_Window *m_window;
        SDL_Renderer *m_renderer;

        std::vector<Circle> m_circlesToRender = {};
};
