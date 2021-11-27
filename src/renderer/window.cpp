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

void Window::pollEvents() 
{
    SDL_Event event;

    while (SDL_PollEvent(&event) != 0)
        onEvent(&event);        
}

void Window::onEvent(SDL_Event *event) 
{
    if (event->type == SDL_QUIT)
        m_isOpened = false;
}

void Window::onLoop() 
{

}

void Window::onRender(Frame frame) 
{
    SDL_SetRenderDrawColor(m_renderer, 10, 10, 30, 255);
    SDL_RenderClear(m_renderer);
    
    for (auto arc : frame.getArcs())
    {
        arc.render(m_renderer);
    };

    for (auto circleBorder : frame.getCirclesBorder())
    {
        circleBorder.render(m_renderer);
    };

    for (auto line : frame.getLines())
    {
        line.render(m_renderer);
    };

    for (auto circle : frame.getCircles())
    {
        circle.render(m_renderer);
    };

    SDL_RenderPresent(m_renderer);

    SDL_Delay(50);
}

void Window::onExit() 
{
    SDL_DestroyWindow(m_window);
    m_window = NULL;

    SDL_Quit();
}
