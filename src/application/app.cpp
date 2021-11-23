#include "app.h"
#include "robotArmExample.h"

int App::run()
{
    RevoluteRevolute robot = RevoluteRevolute(100.f, 50.f, 90.f, 45.f);

    std::vector<Vector3d> joints = robot.getJoints();


    for (auto joint : joints)
    {
        m_window.drawCircle(m_windowWidth / 2 + joint.x, m_windowHeight / 2 - joint.y, 10, 255, 0, 0);
        joint.print();
    }

    Vector3d endEffector = robot.getEndEffector();
    m_window.drawCircle(m_windowWidth / 2 + endEffector.x, m_windowHeight / 2 - endEffector.y, 10, 255, 0, 0);


    m_window.onExecute();

    return 0;
}