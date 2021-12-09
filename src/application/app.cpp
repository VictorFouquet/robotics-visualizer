#include "app.h"
#include "robotArmExample.h"
#include <algorithm>


int App::run()
{
    init();

    RevoluteRevolute derived = RevoluteRevolute(100.f, 50.f, 0.f, 0.f);
    m_robot = &derived;

    Frame frame = computeFrameComponents();
    m_frames.push_back(frame);

    for (int i = 0; i < 90; i++)
    {
        std::vector<float> rotate = { (float) i, (float)(i) };

        m_robot->rotateJoint(rotate);
        Frame frame = computeFrameComponents();
        m_frames.push_back(frame);
    }

    m_frameToRender++;

    while(m_window.isOpened())
    {
        AppEvent event = { .clickCoord = { -1.f, -1.f }, .keyCode = -1 };
        // Handle events from window
        m_window.pollEvents(event);

        // Frame frame = computeFrameComponents();

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

Frame App::computeFrameComponents()
{
    Frame frame;
    std::vector<Vector3d> joints = m_robot->getJoints();

    frame.addCircle(m_windowWidth / 2, m_windowHeight / 2, 10);

    std::vector<std::pair<Vector3d, Vector3d>> links = m_robot->getLinks();
    Vector3d p1(-0.5f, 0.f, 0.f), p2(0.5f, 0.f, 0.f);

    int i = 1;
    std::vector<float> lengths = {};
    for (auto link : links)
    {
        float deltaX = link.first.x - link.second.x;
        float deltaY = link.first.y - link.second.y;

        float dist = std::sqrt((deltaX * deltaX) + (deltaY * deltaY));
    }
    Vector3d endEffector = m_robot->getEndEffector();
    
    std::vector<float> values = m_robot->inverseKinematics(endEffector.x, endEffector.y);
    
    frame.addMessage("a1: " + std::to_string(values[0]), "roboto.ttf", 100, 100 + i * 15, 12);
    i++;
    frame.addMessage("a2: " + std::to_string(values[1]), "roboto.ttf", 100, 100 + i * 15, 12);
    i++;

    m_robot->rotateJoint({ values[0], values[1]});
    joints = m_robot->getJoints();
    frame.addCircle(m_windowWidth / 2 + joints[1].x, m_windowHeight / 2 - joints[1].y, 10);

    endEffector = m_robot->getEndEffector();
    frame.addCircle(m_windowWidth / 2 + endEffector.x, m_windowHeight / 2 - endEffector.y, 10);
    frame.addLine(
        m_windowWidth / 2, m_windowHeight / 2,
        m_windowWidth / 2 + joints[1].x, m_windowHeight / 2 - joints[1].y
    );
    frame.addLine(
        m_windowWidth / 2 + joints[1].x, m_windowHeight / 2 - joints[1].y,
        m_windowWidth / 2 + endEffector.x, m_windowHeight / 2 - endEffector.y
    );

    frame.addCircleBorder(m_windowWidth / 2, m_windowHeight / 2, 150.f, 100.f);


    return frame;
}

}