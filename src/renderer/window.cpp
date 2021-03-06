#include "window.h"
#include "circle.h"
#include "line.h"
#include "messageBox.h"
#include "polygon.h"
#include "triangle.h"
#include "vertex.h"
#include <math.h>
#include <SDL2/SDL_ttf.h>

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
    TTF_Init();

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

void Window::pollEvents(AppEvent& e) 
{
    SDL_Event event;

    while (SDL_PollEvent(&event) != 0)
        onEvent(&event, e);        
}

void Window::onEvent(SDL_Event *event, AppEvent& appEvent) 
{
    if (event->type == SDL_QUIT)
        m_isOpened = false;
    else if (event->type == SDL_MOUSEBUTTONDOWN)
    {
        appEvent.clickCoord.first = event->button.x;
        appEvent.clickCoord.second = event->button.y;
    }
}

void Window::onLoop() 
{

}

void Window::onRender(Frame frame) 
{

    SDL_SetRenderDrawColor(m_renderer, 10, 10, 30, 255);
    SDL_RenderClear(m_renderer);
    
    for (auto rectangle : frame.getRectangles())
        rectangle.render(m_renderer);
        
    for (auto arc : frame.getArcs())
    {
        arc.render(m_renderer);
    };

    for (auto circleBorder : frame.getCirclesBorder())
    {
        circleBorder.render(m_renderer);
    };


    for (auto circle : frame.getCircles())
    {
        circle.render(m_renderer);
    };


    for (auto message : frame.getMessages())
        message.render(m_renderer);

    for (auto line : frame.getLines())
        line.render(m_renderer);
    
    SDL_RenderPresent(m_renderer);
    
    SDL_Delay(50);
}

void Window::onExit() 
{
    SDL_DestroyWindow(m_window);
    m_window = NULL;
    TTF_Quit();
    SDL_Quit();
}
