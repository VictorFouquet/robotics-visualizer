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

    void createRobots();
    void createRR();
    void createRP();
    void createPR();

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
    PrismaticRevolute m_derivedPR = PrismaticRevolute();
    RevolutePrismatic m_derivedRP = RevolutePrismatic();
    RevoluteRevolute m_derivedRR  = RevoluteRevolute();

    std::vector<Vector3d> m_RRTarget = { Vector3d(), Vector3d(14.f, 14.f), Vector3d() };

    bool m_RRActivated = false;
    bool m_RPActivated = false;
    bool m_PRActivated = false;
};
