#include "app.h"
#include "robotArmExample.h"

int App::run()
{
    init();

    RevoluteRevolute derived = RevoluteRevolute(100.f, 50.f, 0.f, 0.f);
    m_robot = &derived;


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
        // Handle events from window
        m_window.pollEvents();

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

    for (auto joint : joints)
        frame.addCircle(m_windowWidth / 2 + joint.x, m_windowHeight / 2 - joint.y, 10);

    Vector3d endEffector = m_robot->getEndEffector();
    frame.addCircle(m_windowWidth / 2 + endEffector.x, m_windowHeight / 2 - endEffector.y, 10);

    std::vector<std::pair<Vector3d, Vector3d>> links = m_robot->getLinks();
    Vector3d p1(-0.5f, 0.f, 0.f), p2(0.5f, 0.f, 0.f);

    for (auto link : links)
    {
        frame.addLine(
            m_windowWidth / 2 + (int)link.first.x, m_windowHeight / 2 - (int)link.first.y,
            m_windowWidth / 2 + (int)link.second.x, m_windowHeight / 2 - (int)link.second.y
        );
    }

    frame.addCircleBorder(m_windowWidth / 2, m_windowHeight / 2, 150.f, 100.f);

    return frame;
}