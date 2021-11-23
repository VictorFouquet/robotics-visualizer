#pragma once

#include "window.h"

class App
{
public:
    App() = default;
    ~App() = default;

    int run();

private:
    int m_windowWidth = 640, m_windowHeight = 480;
    Window m_window =  Window(m_windowWidth, m_windowHeight);
};
