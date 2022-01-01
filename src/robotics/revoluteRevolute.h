#pragma once

#include "robotArm.h"
#include <memory>

class RevoluteRevolute : public RobotArm
{
public:
    RevoluteRevolute() = default;
    RevoluteRevolute(
        std::vector<std::shared_ptr<ArmComponent>> components,
        float weightLink1=1.f, float weightLink2=1.f
    );
    ~RevoluteRevolute() = default;

    virtual void actuateJoints(std::vector<float>) override;

    void rotateLink(int link, int theta);
    virtual std::vector<float> inverseKinematics(float x, float y) override;
    //
    virtual std::vector<std::vector<float>> inverseKinematics(float x, float y, float rotz) override;
    virtual std::vector<std::vector<float>> getDeltasBetweenPoses(float x, float y) override;
    //
    virtual std::vector<std::vector<float>> getDeltasBetweenPoses(float x, float y, float rotz) override;

    virtual std::vector<std::vector<Vector3d>> interpolate(float x, float y, int step) override;
    //
    virtual std::vector<std::vector<Vector3d>> interpolate(Vector3d pos, float rot, int step) override;

    virtual std::vector<std::shared_ptr<ArmComponent>> getRigidBodies() override { return { m_rigidBodies[0], m_rigidBodies[1] }; }
    virtual std::vector<std::shared_ptr<ArmComponent>> getJointComponents() override { return m_jointComponents; } 
    virtual std::shared_ptr<ArmComponent> getEndEffectorComponent() override { return m_endEffComponent; }

private:
    static bool compareDelta(std::vector<float> a, std::vector<float> b);
private:
    float m_theta, m_phi, m_weightLink1=20.f, m_weightLink2=1.f;
};