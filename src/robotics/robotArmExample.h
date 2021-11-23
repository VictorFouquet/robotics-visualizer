#pragma once

#include "vector3d.h"
#include "matrix.h"
#include <iostream>
#include <stdlib.h>

class RobotArm
{
public:
    static void forwardKinematic2DOF_DEMO();
    static void forwardKinematics6DOF(float theta1, float theta2, float theta3, float theta4, float theta5, float theta6);
};
