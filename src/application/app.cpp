#include "app.h"
#include "robotArmExample.h"
#include <algorithm>


int App::run()
{
    init();

    RevoluteRevolute derived = RevoluteRevolute(100.f, 50.f, 0.f, 0.f, 20.f, 1.f);
    m_robot = &derived;
    std::vector<Vector3d> baseFrame = { 
        Vector3d(0.f, 0.f, 0.f),
        Vector3d(100.f, 0.f, 0.f),
        Vector3d(150.f, 0.f, 0.f),
        Vector3d(0.f, 0.f, 0.f)
    };
    Frame frame = computeFrameComponents(baseFrame);
    m_frames.push_back(frame);

    m_frameToRender++;

    while(m_window.isOpened())
    {
        AppEvent event = { .clickCoord = { -1.f, -1.f }, .keyCode = -1 };
        // Handle events from window
        m_window.pollEvents(event);

        if (event.clickCoord.first > -1.f)
        {
            Vector3d point(event.clickCoord.first, event.clickCoord.second, 0.f);
            handleClick(point);
        }

        if (m_window.isOpened())
        {
            m_window.onLoop();
            m_window.onRender(m_frames[m_frameToRender]);

            if (m_frameToRender < m_frames.size() - 1)
                m_frameToRender++;
        }
    }

    return 0;
}

int App::init() 
{
    m_window =  Window(m_windowWidth, m_windowHeight);
    m_window.onInit();
    
    return 0;
}

Frame App::computeFrameComponents(std::vector<Vector3d> step)
{
    Frame frame;
    frame.addCircle(m_windowWidth / 2 + step[0].x, m_windowHeight / 2 + step[0].y, 10);
    frame.addCircle(m_windowWidth / 2 + step[1].x, m_windowHeight / 2 - step[1].y, 10);
    frame.addCircle(m_windowWidth / 2 + step[2].x, m_windowHeight / 2 - step[2].y, 10);
    frame.addLine(
        m_windowWidth / 2 + step[0].x, m_windowHeight / 2 + step[0].x,
        m_windowWidth / 2 + step[1].x, m_windowHeight / 2 - step[1].y
    );
    frame.addLine(
        m_windowWidth / 2 + step[1].x, m_windowHeight / 2 - step[1].y,
        m_windowWidth / 2 + step[2].x, m_windowHeight / 2 - step[2].y
    );

    int i = 1;
    frame.addMessage("a1: " + std::to_string(step[3].x), "roboto.ttf", 100, 100 + i * 15, 12);
    i++;
    frame.addMessage("a2: " + std::to_string(step[3].y), "roboto.ttf", 100, 100 + i * 15, 12);
    i++;

    frame.addCircleBorder(m_windowWidth / 2, m_windowHeight / 2, 150.f, 100.f);

    return frame;
}

void App::handleClick(Vector3d point) 
{
    float deltaX = point.x - m_windowWidth / 2;
    float deltaY = point.y - m_windowHeight / 2;

    float dist = std::sqrt(deltaX * deltaX + deltaY * deltaY);
    if (dist > 50.f && dist < 150.f)
    {
        std::vector<std::vector<Vector3d>> stepsToRender = m_robot->interpolate(deltaX, -deltaY, 5);
        for (auto step : stepsToRender)
        {
            Frame frame = computeFrameComponents(step);
            m_frames.push_back(frame);
        }
        std::cout<<std::endl;
    }
}