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
    Vector3d getEndEffector() const { return m_endEffector; }

    std::vector<std::pair<Vector3d, Vector3d>> getLinks() const { return m_links; }
    std::vector<Vector3d> getJoints() const { return m_joints; }

    virtual void actuateJoints(std::vector<float>) {};
    virtual std::vector<float> inverseKinematics(float x, float y) { return {}; }
    virtual std::vector<float> getRotations() const { return m_rotations; }
    virtual std::vector<std::vector<float>> getDeltasBetweenPoses(float x, float y) { return {}; }
    virtual std::vector<std::vector<Vector3d>> interpolate(float x, float y, int step) { return {}; }

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
    RevoluteRevolute(float lenghtLink1, float lenghtLink2, float theta, float phi, float weightLink1=1.f, float weightLink2=1.f);
    ~RevoluteRevolute() = default;

    virtual void actuateJoints(std::vector<float>) override;

    void rotateLink(int link, int theta);
    virtual std::vector<float> inverseKinematics(float x, float y) override;
    virtual std::vector<std::vector<float>> getDeltasBetweenPoses(float x, float y) override;
    virtual std::vector<std::vector<Vector3d>> interpolate(float x, float y, int step) override;

private:
    static bool compareDelta(std::vector<float> a, std::vector<float> b);
private:
    float m_lenghtLink1, m_lenghtLink2, m_theta, m_phi, m_weightLink1, m_weightLink2;
};

class RevolutePrismatic : public RobotArm
{
public:
    RevolutePrismatic(float lenghtLink1, float lenghtLink2, float phi, float delta, float weightLink1=1.f, float weightLink2=1.f);
    ~RevolutePrismatic() = default;

    virtual void actuateJoints(std::vector<float>) override;

    virtual std::vector<float> inverseKinematics(float x, float y) override;
    virtual std::vector<std::vector<float>> getDeltasBetweenPoses(float x, float y) override;
    virtual std::vector<std::vector<Vector3d>> interpolate(float x, float y, int step) override;

private:
    static bool compareDelta(std::vector<float> a, std::vector<float> b);
private:
    float m_lenghtLink1, m_lenghtLink2, m_phi, m_delta, m_weightLink1, m_weightLink2;
};