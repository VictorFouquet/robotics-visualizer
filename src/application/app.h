#pragma once

#include "gui.h"
#include "window.h"

#include "robotArm.h"
#include "prismaticRevolute.h"
#include "revoluteRevolute.h"
#include "revolutePrismatic.h"

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
    void computeRobotBaseFrame();
    void handleClick(Vector3d point);

    Frame computeRRFrame(std::vector<Vector3d> step, Frame frame) ;
    void handleRRClick(float x, float y);
    
    Frame computeRPFrame(std::vector<Vector3d> step, Frame frame) ;
    void handleRPClick(float x, float y);

    Frame computePRFrame(std::vector<Vector3d> step, Frame frame);
    void handlePRClick(float x, float y);

    void updateRobot();
    void updateRR();
    void updateRP();
    void updatePR();

private:
    int m_windowWidth = 800, m_windowHeight = 600;
    Window m_window =  Window(m_windowWidth, m_windowHeight);
    bool m_windowOpened = true;

    std::vector<Frame> m_frames;
    int m_frameToRender = -1;
    
    GUI m_gui = GUI();
    RobotArm* m_robot;
    PrismaticRevolute m_derivedPR = PrismaticRevolute(120.f, 40.f, 10.f, 0.f, 80.f, 0.f, 200.f, 1.f);
    RevolutePrismatic m_derivedRP = RevolutePrismatic(75.f, 10.f, 0.f, 0.f, 0.f, 0.f, 1.f, 1.f);
    RevoluteRevolute m_derivedRR  = RevoluteRevolute(100.f, 50.f, 0.f, 0.f, 0.f, 0.f, 20.f, 1.f);

    bool m_RRActivated = false;
    bool m_RPActivated = false;
    bool m_PRActivated = false;
};
