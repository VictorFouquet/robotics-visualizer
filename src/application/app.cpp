#include "app.h"
#include "robotArm.h"
#include <algorithm>
#include <memory>

int App::run()
{
    init();
    m_gui.init(m_windowWidth, m_windowHeight);
    createGUI();
    createRobots();
    
    while(m_window.isOpened())
    {
        updateRobot();

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
    robot3Btn.setCallback([this]() mutable { this->m_view = 3; });

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
    else if (m_view == 3)
        return computePRFrame(step, frame);
    
    return frame;
}

void App::computeRobotBaseFrame() 
{
    std::shared_ptr<ArmComponent> cmp = m_robot->getJointComponents()[0];
    Vector3d joint1 = m_robot->getJointComponents()[0]->getTransformedPoints()[0];
    Vector3d jointsValues;
    std::vector<Vector3d> baseFrame = {};

    if (m_view == 1)
    {
        baseFrame = { 
            m_robot->getJointComponents()[0]->getTransformedPoints()[0],
            m_robot->getJointComponents()[1]->getTransformedPoints()[0]
        };
        std::vector<Vector3d> endEff = m_robot->getEndEffectorComponent()->getTransformedPoints();
        for (auto p : endEff)
            baseFrame.push_back(p);
        baseFrame.push_back(
            Vector3d(
                m_robot->getJointComponents()[0]->getRotation().z,
                m_robot->getJointComponents()[1]->getRotation().z
            )
        );
    }
    else if (m_view == 2)
    {
        baseFrame = { 
            m_robot->getJointComponents()[0]->getTransformedPoints()[0],
            m_robot->getJointComponents()[1]->getTransformedPoints()[0],
            m_robot->getEndEffectorComponent()->getTransformedPoints()[0],
            Vector3d(
                m_robot->getJointComponents()[0]->getRotation().z,
                m_robot->getJointComponents()[1]->getTranslation().x
            )
        };
    }
    else if (m_view == 3)
    {
        baseFrame = {
            Vector3d(),
            m_robot->getJointComponents()[1]->getTransformedPoints()[0],
            m_robot->getEndEffectorComponent()->getTransformedPoints()[0],
            Vector3d(
                m_robot->getJointComponents()[0]->getTranslation().x,
                m_robot->getJointComponents()[1]->getRotation().z
            )
        };
    }

    Frame frame = computeFrameComponents(baseFrame);

    m_frames.push_back(frame);

    m_frameToRender++;
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

    std::vector<Vector3d> endEffPoints = m_robot->getEndEffectorComponent()->getTransformedPoints();
    for (int i = 2; i < step.size() - 1; i+=2)
        frame.addLine(
            m_windowWidth / 2 + step[i].x, m_windowHeight / 2 - step[i].y,
            m_windowWidth / 2 + step[i+1].x, m_windowHeight / 2 - step[i+1].y
        );

    int i = 1;
    frame.addMessage("a1: " + std::to_string(step.back().x), "roboto.ttf", 500, 100 + i * 15, 12);
    i++;
    frame.addMessage("a2: " + std::to_string(step.back().y), "roboto.ttf", 500, 100 + i * 15, 12);
    i++;

    frame.addCircleBorder(m_windowWidth / 2, m_windowHeight / 2, 150.f, 100.f);

    frame.addRectangle(20, 20, 300, 100, 255, 255, 255);
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
    
    int i = 1;
    frame.addMessage("a1: " + std::to_string(step[3].x), "roboto.ttf", 500, 100 + i * 15, 12);
    i++;
    frame.addMessage("a2: " + std::to_string(step[3].y), "roboto.ttf", 500, 100 + i * 15, 12);
    i++;
    
    frame.addCircleBorder(m_windowWidth / 2, m_windowHeight / 2, 150.f, 65.f);
    
    return frame;
}

Frame App::computePRFrame(std::vector<Vector3d> step, Frame frame)
{
    std::vector<float> linksLengths = m_robot->getLinksLengths();
    std::vector<float> maxJoints = m_robot->getMaxJointValues();

    // Draws legal area
    frame.addCircle(
        m_windowWidth / 2, m_windowHeight / 2 - linksLengths[0] - 10.f, 
        linksLengths[1],
        50.f, 150.f, 50.f
    );
    frame.addCircle(
        m_windowWidth / 2, m_windowHeight / 2 - linksLengths[0] - maxJoints[0],
        linksLengths[1],
        50.f, 150.f, 50.f
    );
    frame.addRectangle(
        linksLengths[1] * 2, maxJoints[0], m_windowWidth / 2 - linksLengths[1],
        m_windowHeight / 2 - linksLengths[0] - maxJoints[0],
        50.f, 150.f, 50.f
    );

    frame.addCircle(m_windowWidth / 2 + step[0].x, m_windowHeight / 2 + step[0].y, 10);

    frame.addCircle(m_windowWidth / 2 + step[1].x, m_windowHeight / 2 - step[1].y, 10);

    frame.addCircle(m_windowWidth / 2 + step[2].x, m_windowHeight / 2 - step[2].y, 10);

    frame.addLine(
        m_windowWidth / 2, m_windowHeight / 2,
        m_windowWidth / 2, m_windowHeight / 2 - step[1].y
    );
    frame.addLine(
        m_windowWidth / 2 + step[1].x, m_windowHeight / 2 - step[1].y,
        m_windowWidth / 2 + step[2].x, m_windowHeight / 2 - step[2].y
    );

    Vector3d basePrismatic = Vector3d(0.f, linksLengths[0]);
    std::vector<Vector3d> prismaticJoint = basePrismatic.normals();

    prismaticJoint[0] = prismaticJoint[0].unit() * 10 + basePrismatic;
    prismaticJoint[1] = prismaticJoint[1].unit() * 10 + basePrismatic;

    frame.addLine(
        m_windowWidth / 2 + prismaticJoint[0].x, m_windowHeight / 2 - prismaticJoint[0].y,
        m_windowWidth / 2 + prismaticJoint[1].x, m_windowHeight / 2 - prismaticJoint[1].y
    );

    int i = 1;
    frame.addMessage("a1: " + std::to_string(step[3].x), "roboto.ttf", 500, 100 + i * 15, 12);
    i++;
    frame.addMessage("a2: " + std::to_string(step[3].y), "roboto.ttf", 500, 100 + i * 15, 12);
    i++;
        
    return frame;
}

void App::handleClick(Vector3d point) 
{
    float deltaX = point.x - m_windowWidth / 2;
    float deltaY = point.y - m_windowHeight / 2;

    if (m_view == 1)
        handleRRClick(deltaX, -deltaY);
    if (m_view == 2)
        handleRPClick(deltaX, -deltaY);
    if (m_view == 3)
        handlePRClick(deltaX, -deltaY);
}

void App::handleRRClick(float x, float y)
{
    float dist = std::sqrt(x * x + y * y);
    if (dist > 50.f && dist < 150.f)
    {
        std::vector<std::vector<Vector3d>> stepsToRender = m_robot->interpolate(Vector3d(x, y), 0.f, 5);
        for (auto step : stepsToRender)
        {
            Frame frame = computeFrameComponents(step);
            m_frames.push_back(frame);
        }
    }
}

void App::handleRPClick(float x, float y) 
{
    float dist = std::sqrt(x * x + y * y);
    if (dist > 85.f && dist < 150.f)
    {
        std::vector<std::vector<Vector3d>> stepsToRender = m_robot->interpolate(x, y, 5);
        for (auto step : stepsToRender)
        {
            Frame frame = computeFrameComponents(step);
            m_frames.push_back(frame);
        }
    }
}

void App::handlePRClick(float x, float y) 
{
    std::vector<float> linksLengths = m_robot->getLinksLengths();
    std::vector<float> maxJoints = m_robot->getMaxJointValues();

    Vector3d clicked = Vector3d(x, y);
    Vector3d c1Center = Vector3d(0.f, linksLengths[0] + 10.f);
    Vector3d c2Center = Vector3d(0.f, linksLengths[0] + maxJoints[0]);

    bool inRect = (
        clicked.x > -linksLengths[1] && clicked.x < linksLengths[1] &&
        clicked.y > linksLengths[0] + 10.f && clicked.y < linksLengths[0] + maxJoints[0]
    );

    float d1 = clicked.distanceToVector(c1Center);
    float d2 = clicked.distanceToVector(c2Center);

    if (d1 < linksLengths[1] || d2 < linksLengths[1] || inRect)
    {
        std::vector<std::vector<Vector3d>> stepsToRender = m_robot->interpolate(x, y, 5);
        for (auto step : stepsToRender)
        {
            Frame frame = computeFrameComponents(step);
            m_frames.push_back(frame);
        }
    }
}

void App::updateRobot() 
{
    if (m_view == 1)
        updateRR();
    else if (m_view == 2)
       updateRP();
    else if (m_view == 3)
        updatePR();
}

void App::updateRR() 
{
    m_robot = &m_derivedRR;
    if (!m_RRActivated)
    {
        m_RRActivated = true;
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
    else
        computeRobotBaseFrame();
}

void App::updateRP() 
{
    m_robot = &m_derivedRP;
    if (!m_RPActivated)
    {
        m_RPActivated = true;
        std::vector<Vector3d> baseFrame = { 
            Vector3d(0.f, 0.f, 0.f),
            Vector3d(75.f, 0.f, 0.f),
            Vector3d(85.f, 0.f, 0.f)
        };
        Frame frame = computeFrameComponents(baseFrame);


        m_frames.push_back(frame);

        m_frameToRender++;
    }
    else
        computeRobotBaseFrame();
}

void App::updatePR() 
{
    m_robot = &m_derivedPR;
    if (!m_PRActivated)
    {
        m_PRActivated = true;
        std::vector<Vector3d> baseFrame = {
            Vector3d(),
            Vector3d(0.f, 130.f, 0.f),
            Vector3d(0.f, 170.f, 0.f),
            Vector3d(0.f, 10.f, 0.f)
        };
        Frame frame = computeFrameComponents(baseFrame);


        m_frames.push_back(frame);

        m_frameToRender++;
    }
    else
        computeRobotBaseFrame();
}

void App::createRobots() 
{
    createRR();
    createRP();
    createPR();
}

void App::createRR()
{
    ArmComponent ground = ArmComponent(
        Vector3d(),
        Vector3d(),
        { Vector3d() },
        ArmComponentType::ground
    );
    
    ArmComponent joint1 = ArmComponent(
        Vector3d(),
        Vector3d(0.f, 0.f, 0.f),
        { Vector3d() },
        ArmComponentType::revolute
    );

    ArmComponent link1  = ArmComponent(
        Vector3d(100.f, 0.f),
        Vector3d(),
        { Vector3d(), Vector3d(100.f, 0.f) },
        ArmComponentType::rigidBody
    );

    ArmComponent joint2 = ArmComponent(
        Vector3d(),
        Vector3d(),
        { Vector3d() },
        ArmComponentType::revolute
    );

    ArmComponent link2  = ArmComponent(
        Vector3d(50.f, 0.f),
        Vector3d(),
        { Vector3d(), Vector3d(50.f, 0.f) },
        ArmComponentType::rigidBody
    );
    
    ArmComponent joint3 = ArmComponent(
        Vector3d(),
        Vector3d(),
        { Vector3d() },
        ArmComponentType::revolute
    );

    ArmComponent endEff = ArmComponent(
        Vector3d(),
        Vector3d(),
        {
            Vector3d(), Vector3d(20.f, 0.f),
            Vector3d(20.f, 10.f), Vector3d(20.f, -10.f),
            Vector3d(20.f, 10.f), Vector3d(30.f, 10.f),
            Vector3d(20.f, -10.f), Vector3d(30.f, -10.f)
        },
        ArmComponentType::endEffector
    );

    std::shared_ptr<ArmComponent> groundPtr = std::make_shared<ArmComponent>(ground);
    std::shared_ptr<ArmComponent> joint1Ptr = std::make_shared<ArmComponent>(joint1);
    std::shared_ptr<ArmComponent> link1Ptr = std::make_shared<ArmComponent>(link1);
    std::shared_ptr<ArmComponent> joint2Ptr = std::make_shared<ArmComponent>(joint2);
    std::shared_ptr<ArmComponent> link2Ptr = std::make_shared<ArmComponent>(link2);
    std::shared_ptr<ArmComponent> joint3Ptr = std::make_shared<ArmComponent>(joint3);
    std::shared_ptr<ArmComponent> endEffPtr = std::make_shared<ArmComponent>(endEff);

    groundPtr->setChild(joint1Ptr);
    joint1Ptr->setParent(groundPtr);
    joint1Ptr->setChild(link1Ptr);
    link1Ptr->setParent(joint1Ptr);
    link1Ptr->setChild(joint2Ptr);
    joint2Ptr->setParent(link1Ptr);
    joint2Ptr->setChild(link2Ptr);
    link2Ptr->setParent(joint2Ptr);
    link2Ptr->setChild(joint3Ptr);
    joint3Ptr->setParent(link2Ptr);
    joint3Ptr->setChild(endEffPtr);
    endEffPtr->setParent(joint3Ptr);

    m_derivedRR = RevoluteRevolute({
        groundPtr, joint1Ptr, link1Ptr, joint2Ptr, link2Ptr, joint3Ptr, endEffPtr
    }, 20.f, 1.f);
}

void App::createRP()
{
    ArmComponent ground = ArmComponent(
        Vector3d(),
        Vector3d(),
        { Vector3d() },
        ArmComponentType::ground
    );
    
    ArmComponent joint1 = ArmComponent(
        Vector3d(),
        Vector3d(0.f, 0.f, 0.f),
        { Vector3d() },
        ArmComponentType::revolute
    );

    ArmComponent link1  = ArmComponent(
        Vector3d(75.f, 0.f),
        Vector3d(),
        { Vector3d(), Vector3d(75.f, 0.f) },
        ArmComponentType::rigidBody
    );

    ArmComponent joint2 = ArmComponent(
        Vector3d(),
        Vector3d(),
        { Vector3d() },
        ArmComponentType::prismatic
    );

    ArmComponent link2  = ArmComponent(
        Vector3d(10.f, 0.f),
        Vector3d(),
        { Vector3d(), Vector3d(10.f, 0.f) },
        ArmComponentType::rigidBody
    );
    
    ArmComponent joint3 = ArmComponent(
        Vector3d(),
        Vector3d(),
        { Vector3d() },
        ArmComponentType::fixed
    );

    ArmComponent endEff = ArmComponent(
        Vector3d(),
        Vector3d(),
        { Vector3d() },
        ArmComponentType::endEffector
    );

    std::shared_ptr<ArmComponent> groundPtr = std::make_shared<ArmComponent>(ground);
    std::shared_ptr<ArmComponent> joint1Ptr = std::make_shared<ArmComponent>(joint1);
    std::shared_ptr<ArmComponent> link1Ptr  = std::make_shared<ArmComponent>(link1);
    std::shared_ptr<ArmComponent> joint2Ptr = std::make_shared<ArmComponent>(joint2);
    std::shared_ptr<ArmComponent> link2Ptr  = std::make_shared<ArmComponent>(link2);
    std::shared_ptr<ArmComponent> joint3Ptr = std::make_shared<ArmComponent>(joint3);
    std::shared_ptr<ArmComponent> endEffPtr = std::make_shared<ArmComponent>(endEff);

    groundPtr->setChild(joint1Ptr);
    joint1Ptr->setParent(groundPtr);
    joint1Ptr->setChild(link1Ptr);
    link1Ptr->setParent(joint1Ptr);
    link1Ptr->setChild(joint2Ptr);
    joint2Ptr->setParent(link1Ptr);
    joint2Ptr->setChild(link2Ptr);
    link2Ptr->setParent(joint2Ptr);
    link2Ptr->setChild(joint3Ptr);
    joint3Ptr->setParent(link2Ptr);
    joint3Ptr->setChild(endEffPtr);
    endEffPtr->setParent(joint3Ptr);

    m_derivedRP = RevolutePrismatic({
        groundPtr, joint1Ptr, link1Ptr, joint2Ptr, link2Ptr, joint3Ptr, endEffPtr
    }, 20.f, 1.f);
}

void App::createPR()
{
    ArmComponent ground = ArmComponent(
        Vector3d(),
        Vector3d(),
        { Vector3d() },
        ArmComponentType::ground
    );

    ArmComponent link1  = ArmComponent(
        Vector3d(120.f, 0.f),
        Vector3d(0.f, 0.f, 90.f),
        { Vector3d(), Vector3d(75.f, 0.f) },
        ArmComponentType::rigidBody
    );

    ArmComponent joint1 = ArmComponent(
        Vector3d(10.f, 0.f),
        Vector3d(),
        { Vector3d() },
        ArmComponentType::prismatic
    );

    ArmComponent joint2 = ArmComponent(
        Vector3d(),
        Vector3d(0.f, 0.f, 0.f),
        { Vector3d() },
        ArmComponentType::revolute
    );

    ArmComponent link2  = ArmComponent(
        Vector3d(40.f, 0.f),
        Vector3d(),
        { Vector3d(), Vector3d(10.f, 0.f) },
        ArmComponentType::rigidBody
    );

    ArmComponent joint3 = ArmComponent(
        Vector3d(),
        Vector3d(),
        { Vector3d() },
        ArmComponentType::fixed
    );

    ArmComponent endEff = ArmComponent(
        Vector3d(),
        Vector3d(),
        { Vector3d() },
        ArmComponentType::endEffector
    );

    std::shared_ptr<ArmComponent> groundPtr = std::make_shared<ArmComponent>(ground);
    std::shared_ptr<ArmComponent> joint1Ptr = std::make_shared<ArmComponent>(joint1);
    std::shared_ptr<ArmComponent> link1Ptr  = std::make_shared<ArmComponent>(link1);
    std::shared_ptr<ArmComponent> joint2Ptr = std::make_shared<ArmComponent>(joint2);
    std::shared_ptr<ArmComponent> link2Ptr  = std::make_shared<ArmComponent>(link2);
    std::shared_ptr<ArmComponent> joint3Ptr = std::make_shared<ArmComponent>(joint3);
    std::shared_ptr<ArmComponent> endEffPtr = std::make_shared<ArmComponent>(endEff);

    groundPtr->setChild(joint1Ptr);
    link1Ptr->setParent(groundPtr);
    link1Ptr->setChild(joint1Ptr);
    joint1Ptr->setParent(link1Ptr);
    joint1Ptr->setChild(joint2Ptr);
    joint2Ptr->setParent(joint1Ptr);
    joint2Ptr->setChild(link2Ptr);
    link2Ptr->setParent(joint2Ptr);
    link2Ptr->setChild(joint3Ptr);
    joint3Ptr->setParent(link2Ptr);
    joint3Ptr->setChild(endEffPtr);
    endEffPtr->setParent(joint3Ptr);

    m_derivedPR = PrismaticRevolute({
        groundPtr, link1Ptr, joint1Ptr, joint2Ptr, link2Ptr, joint3Ptr, endEffPtr
    }, 20.f, 1.f);
}