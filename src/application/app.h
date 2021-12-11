#pragma once

#include "window.h"
#include "robotArmExample.h"

#include <vector>


class App
{
public:
    App() = default;
    ~App() = default;

    int run();

private:
    int init();
    Frame computeFrameComponents(std::vector<Vector3d> step);
private:
    int m_windowWidth = 640, m_windowHeight = 480;
    Window m_window =  Window(m_windowWidth, m_windowHeight);
    bool m_windowOpened = true;

    std::vector<Frame> m_frames;
    int m_frameToRender = -1;
    
    RobotArm* m_robot;
};
