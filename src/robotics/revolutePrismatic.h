#pragma once

#include "robotArm.h"

class RevolutePrismatic : public RobotArm
{
public:
    RevolutePrismatic() = default;
    RevolutePrismatic(
        std::vector<std::shared_ptr<ArmComponent>> components,
        float weightLink1=1.f, float weightLink2=1.f
    );
    ~RevolutePrismatic() = default;

    virtual std::vector<std::shared_ptr<ArmComponent>> getRigidBodies() override { return { m_rigidBodies[0], m_rigidBodies[1] }; }
    virtual std::vector<std::shared_ptr<ArmComponent>> getJointComponents() override { return m_jointComponents; } 
    virtual std::shared_ptr<ArmComponent> getEndEffectorComponent() override { return m_endEffComponent; }

    virtual void actuateJoints(std::vector<float>) override;

    virtual std::vector<float> inverseKinematics(float x, float y) override;
    virtual std::vector<std::vector<float>> getDeltasBetweenPoses(float x, float y) override;
    virtual std::vector<std::vector<Vector3d>> interpolate(float x, float y, int step) override;

private:
    static bool compareDelta(std::vector<float> a, std::vector<float> b);
private:
    float m_phi, m_delta, m_weightLink1, m_weightLink2;
};