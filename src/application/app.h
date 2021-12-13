#pragma once

#include "gui.h"
#include "window.h"
#include "robotArmExample.h"
#include "uiComponent.h"
#include <vector>


class App
{
public:
    App() = default;
    ~App() = default;

    int run();
    void createGUI();
    int m_view = 0;
private:
    int init();
    Frame computeFrameComponents(std::vector<Vector3d> step);
    void handleClick(Vector3d point);

    Frame computeRRFrame(std::vector<Vector3d> step, Frame frame) ;
    void handleRRClick(float x, float y);
private:
    int m_windowWidth = 800, m_windowHeight = 600;
    Window m_window =  Window(m_windowWidth, m_windowHeight);
    bool m_windowOpened = true;

    std::vector<Frame> m_frames;
    int m_frameToRender = -1;
    
    GUI m_gui = GUI();
    RobotArm* m_robot;
};
