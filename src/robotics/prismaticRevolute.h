#pragma once

#include "robotArm.h"
#include "circle.h"

class PrismaticRevolute : virtual public RobotArm
{
public:
    PrismaticRevolute(float lenghtLink1, float lenghtLink2, float delta, float phi,
        float maxJoint1=1.f, float maxJoint2=1.f, float weightLink1=1.f, float weightLink2=1.f);
    ~PrismaticRevolute() = default;

    virtual void actuateJoints(std::vector<float>) override;

    virtual std::vector<float> inverseKinematics(float x, float y) override;
    virtual std::vector<std::vector<float>> getDeltasBetweenPoses(float x, float y) override;
    virtual std::vector<std::vector<Vector3d>> interpolate(float x, float y, int step) override;

private:
    static bool compareDelta(std::vector<float> a, std::vector<float> b);
private:
    float m_delta, m_phi, m_weightLink1, m_weightLink2;
};