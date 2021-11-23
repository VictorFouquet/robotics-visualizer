#include "window.h"
#include "circle.h"
#include "line.h"
#include "polygon.h"
#include "triangle.h"
#include "vertex.h"

#include <math.h>

Window::Window(int width, int height) 
{
    m_isOpened = true;
    m_window = NULL;
    m_renderer = NULL;
    m_width = width;
    m_height = height;  
}

int Window::onExecute() 
{
    SDL_Event event;

    if (onInit() == false)
        return -1;

    while (m_isOpened) 
    {
        while (SDL_PollEvent(&event) != 0)
            onEvent(&event);
        
        onLoop();
        onRender();
    }

    return 0;
}

bool Window::onInit() 
{
    SDL_Init(SDL_INIT_VIDEO);

    m_window = SDL_CreateWindow(
        "Robotics Visualizer",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        m_width,
        m_height,
        SDL_WINDOW_OPENGL
    );

    if (m_window == NULL) {
        printf("Could not create window: %s\n", SDL_GetError());
        return false;
    }

    m_renderer = SDL_CreateRenderer(m_window, -1, 0);

    SDL_SetRenderDrawColor(m_renderer, 10, 10, 30, 255);
    SDL_RenderClear(m_renderer);

    SDL_RenderPresent(m_renderer);

    return true;
}

void Window::onEvent(SDL_Event *event) 
{
    if (event->type == SDL_QUIT)
        m_isOpened = false;
}

void Window::onLoop() 
{
    SDL_SetRenderDrawColor(m_renderer, 10, 10, 30, 255);
    SDL_RenderClear(m_renderer);

    for (auto shape : m_circlesToRender)
    {
        shape.render(m_renderer);
    };

    SDL_RenderPresent(m_renderer);
}

void Window::onRender() 
{
    SDL_Delay(10);
}

void Window::onExit() 
{
    SDL_DestroyWindow(m_window);
    m_window = NULL;

    SDL_Quit();
}
