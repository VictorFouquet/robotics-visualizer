#pragma once

#include "window.h"

class App
{
public:
    App() = default;
    ~App() = default;

    int run();

private:
    Window m_window =  Window(640, 480);
};
