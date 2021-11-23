#pragma once

#include "vector3d.h"
#include "matrix.h"
#include <iostream>
#include <stdlib.h>
#include <vector>

class RobotArm
{
public:
    RobotArm() = default;
    ~RobotArm() = default;

public:
    static void forwardKinematic2DOF_DEMO();
    static void forwardKinematics6DOF(float theta1, float theta2, float theta3, float theta4, float theta5, float theta6);

    Vector3d getEndEffector() const { return m_endEffector; }

protected:
    std::vector<Matrix> m_transforms;
    std::vector<Vector3d> m_joints;
    std::vector<Matrix> m_links;
    Vector3d m_endEffector;
};