#pragma once

#include <iostream>
#include <stdlib.h>
#include <vector>
#include <tuple>
#include <memory>

#include "vector3d.h"
#include "matrix.h"

#include "armComponent.h"


class RobotArm
{
public:
    RobotArm() = default;
    ~RobotArm() = default;

public:
    Vector3d getEndEffector() const { return m_endEffector; }

    std::vector<std::pair<Vector3d, Vector3d>> getLinks() const { return m_links; }
    std::vector<float> getLinksLengths() const { return { m_lengthLink1, m_lengthLink2 }; }
    std::vector<float> getMaxJointValues() const { return { m_maxJoint1, m_maxJoint2 }; }
    std::vector<Vector3d> getJoints() const { return m_joints; }

    virtual void actuateJoints(std::vector<float>) {};
    virtual std::vector<float> inverseKinematics(float x, float y) { return {}; }
    virtual std::vector<float> getRotations() const { return m_rotations; }
    virtual std::vector<std::vector<float>> getDeltasBetweenPoses(float x, float y) { return {}; }
    virtual std::vector<std::vector<Vector3d>> interpolate(float x, float y, int step) { return {}; }

protected:
    std::vector<std::shared_ptr<ArmComponent>> m_components;
    std::vector<std::shared_ptr<ArmComponent>> m_rigidBodies;
    std::vector<std::shared_ptr<ArmComponent>> m_jointComponents;
    std::shared_ptr<ArmComponent> m_endEffComponent;

    float m_lengthLink1, m_lengthLink2;
    float m_maxJoint1, m_maxJoint2;
    
    std::vector<Matrix> m_transforms;
    std::vector<Vector3d> m_joints;
    
    std::vector<std::pair<Vector3d, Vector3d>> m_links;
    std::vector<float> m_rotations;
    
    Vector3d m_endEffector;
};
