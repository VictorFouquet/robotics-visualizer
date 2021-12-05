#pragma once

#include "vector3d.h"
#include "matrix.h"
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <tuple>

class RobotArm
{
public:
    RobotArm() = default;
    ~RobotArm() = default;

public:
    static void forwardKinematic2DOF_DEMO();
    static void forwardKinematics6DOF(float theta1, float theta2, float theta3, float theta4, float theta5, float theta6);

    Vector3d getEndEffector() const { return m_endEffector; }

    std::vector<std::pair<Vector3d, Vector3d>> getLinks() const { return m_links; }
    std::vector<Vector3d> getJoints() const { return m_joints; }

    virtual void rotateJoint(std::vector<float>) {};
    virtual std::vector<float> inverseKinematics(float x, float y) { return {}; }
    virtual std::vector<float> getRotations() const { return m_rotations; }
protected:
    std::vector<Matrix> m_transforms;
    std::vector<Vector3d> m_joints;
    std::vector<std::pair<Vector3d, Vector3d>> m_links;
    std::vector<float> m_rotations;
    Vector3d m_endEffector;
};

class RevoluteRevolute : public RobotArm
{
public:
    RevoluteRevolute(float lenghtLink1, float lenghtLink2, float theta, float phi);
    ~RevoluteRevolute() = default;

    virtual void rotateJoint(std::vector<float>) override;

    void rotateLink(int link, int theta);
    virtual std::vector<float> inverseKinematics(float x, float y) override;

private:
    float m_lenghtLink1, m_lenghtLink2, m_theta, m_phi;
};