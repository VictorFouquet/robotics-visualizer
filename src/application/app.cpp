#include "app.h"
#include "robotArmExample.h"
#include <algorithm>


int App::run()
{
    init();
    m_gui.init(m_windowWidth, m_windowHeight);
    createGUI();


    RevoluteRevolute derivedRR = RevoluteRevolute(100.f, 50.f, 0.f, 0.f, 20.f, 1.f);
    RevolutePrismatic derivedRP = RevolutePrismatic(75.f, 10.f, 0.f, 0.f, 1.f, 1.f);

    while(m_window.isOpened())
    {
        if (m_view == 1)
        {
            m_robot = &derivedRR;
            std::vector<Vector3d> baseFrame = { 
                Vector3d(0.f, 0.f, 0.f),
                Vector3d(100.f, 0.f, 0.f),
                Vector3d(150.f, 0.f, 0.f),
                Vector3d(0.f, 0.f, 0.f)
            };
            Frame frame = computeFrameComponents(baseFrame);


            m_frames.push_back(frame);

            m_frameToRender++;
        }
        else if (m_view == 2)
        {
            m_robot = &derivedRR;
            std::vector<Vector3d> baseFrame = { 
                Vector3d(0.f, 0.f, 0.f),
                Vector3d(75.f, 0.f, 0.f),
                Vector3d(85.f, 0.f, 0.f)
            };
            Frame frame = computeFrameComponents(baseFrame);


            m_frames.push_back(frame);

            m_frameToRender++;
        }

        AppEvent event = { .clickCoord = { -1.f, -1.f }, .keyCode = -1 };
        // Handle events from window
        m_window.pollEvents(event);
        if (event.clickCoord.first > -1.f)
        {
            m_gui.onEvent(event);
            
            Vector3d point(event.clickCoord.first, event.clickCoord.second, 0.f);
            if (m_robot)
                handleClick(point);

            std::vector<UIComponent> btns = m_gui.getButtons(m_gui.getMainContainer());

            for (UIComponent btn : btns)
            {
                btn.onEvent(event);
                if (btn.clicked()) btn.callback();
            }
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

void App::createGUI() 
{
    UIComponent robotTypesMenu = m_gui.createContainer(0, 0, 150, 600);
    
    UIComponent robot1Btn = m_gui.createButton("Revolute-Revolute", 0, 10, 150, 26);
    robot1Btn.setInnerText("Revolute-Revolute");
    robot1Btn.setBorderColor(255, 255, 255, 255);
    robot1Btn.setCallback([this]() mutable { this->m_view = 1; });

    UIComponent robot2Btn = m_gui.createButton("Revolute-Prismatic", 0, 36, 150, 26);
    robot2Btn.setBorderColor(255, 255, 255, 255);
    robot2Btn.setCallback([this]() mutable { this->m_view = 2; });

    UIComponent robot3Btn = m_gui.createButton("Prismatic-Revolute", 0, 62, 150, 26);
    robot3Btn.setBorderColor(255, 255, 255, 255);
    robot3Btn.setCallback([msg=robot2Btn.getInnerText()]() { std::cout << msg << std::endl; });

    robotTypesMenu.appendChild(robot1Btn);
    robotTypesMenu.appendChild(robot2Btn);
    robotTypesMenu.appendChild(robot3Btn);

    m_gui.appendComponent(robotTypesMenu);

    Frame frame;
    std::vector<UIComponent> components = m_gui.getComponents();
    
    for (UIComponent component : components)
        component.render(&frame);
    
    m_frames.push_back(frame);

    m_frameToRender++;
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

    std::vector<UIComponent> components = m_gui.getComponents();
    for (UIComponent component : components)
        component.render(&frame);

    if (m_view == 1)
        return computeRRFrame(step, frame);
    else if (m_view == 2)
        return computeRPFrame(step, frame);
    return frame;
}

Frame App::computeRRFrame(std::vector<Vector3d> step, Frame frame)
{
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
    frame.addMessage("a1: " + std::to_string(step[3].x), "roboto.ttf", 500, 100 + i * 15, 12);
    i++;
    frame.addMessage("a2: " + std::to_string(step[3].y), "roboto.ttf", 500, 100 + i * 15, 12);
    i++;

    frame.addCircleBorder(m_windowWidth / 2, m_windowHeight / 2, 150.f, 100.f);

    return frame;
}

Frame App::computeRPFrame(std::vector<Vector3d> step, Frame frame)
{
    frame.addCircle(m_windowWidth / 2 + step[0].x, m_windowHeight / 2 + step[0].y, 10);

    frame.addCircle(m_windowWidth / 2 + step[2].x, m_windowHeight / 2 - step[2].y, 10);
    frame.addLine(
        m_windowWidth / 2 + step[0].x, m_windowHeight / 2 + step[0].x,
        m_windowWidth / 2 + step[1].x, m_windowHeight / 2 - step[1].y
    );
    frame.addLine(
        m_windowWidth / 2 + step[1].x, m_windowHeight / 2 - step[1].y,
        m_windowWidth / 2 + step[2].x, m_windowHeight / 2 - step[2].y
    );

    std::vector<Vector3d> prismaticJoint = step[1].normals();

    prismaticJoint[0] = prismaticJoint[0].unit() * 10 + step[1];
    prismaticJoint[1] = prismaticJoint[1].unit() * 10 + step[1];

    frame.addLine(
        m_windowWidth / 2 + prismaticJoint[0].x, m_windowHeight / 2 - prismaticJoint[0].y,
        m_windowWidth / 2 + prismaticJoint[1].x, m_windowHeight / 2 - prismaticJoint[1].y
    );

    frame.addCircleBorder(m_windowWidth / 2, m_windowHeight / 2, 150.f, 65.f);
    
    return frame;
}

void App::handleClick(Vector3d point) 
{
    float deltaX = point.x - m_windowWidth / 2;
    float deltaY = point.y - m_windowHeight / 2;

    if (m_view == 1)
        handleRRClick(deltaX, -deltaY);
}

void App::handleRRClick(float x, float y)
{
    float dist = std::sqrt(x * x + y * y);
    if (dist > 50.f && dist < 150.f)
    {
        std::vector<std::vector<Vector3d>> stepsToRender = m_robot->interpolate(x, y, 5);
        for (auto step : stepsToRender)
        {
            Frame frame = computeFrameComponents(step);
            m_frames.push_back(frame);
        }
    }
}
