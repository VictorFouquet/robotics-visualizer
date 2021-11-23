#include "app.h"
#include "robotArmExample.h"

int App::run()
{
    RobotArm simulator;
    simulator.forwardKinematic2DOF_DEMO();

    m_window.onExecute();

    return 0;
}