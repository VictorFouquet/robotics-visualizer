#pragma once

#include "robotArm.h"
#include <memory>

class RevoluteRevolute : public RobotArm
{
public:
    RevoluteRevolute(float lenghtLink1, float lenghtLink2, float theta, float phi,
        float maxJoint1=1.f, float maxJoint2=1.f, float weightLink1=1.f, float weightLink2=1.f);
    RevoluteRevolute(std::vector<std::shared_ptr<ArmComponent>> components);
    ~RevoluteRevolute() = default;

    virtual void actuateJoints(std::vector<float>) override;

    void rotateLink(int link, int theta);
    virtual std::vector<float> inverseKinematics(float x, float y) override;
    virtual std::vector<std::vector<float>> getDeltasBetweenPoses(float x, float y) override;
    virtual std::vector<std::vector<Vector3d>> interpolate(float x, float y, int step) override;
    
    std::vector<std::shared_ptr<ArmComponent>> getRigidBodies() { return m_rigidBodies; }
    std::vector<std::shared_ptr<ArmComponent>> getJointComponents() { return m_jointComponents; } 
private:
    static bool compareDelta(std::vector<float> a, std::vector<float> b);
private:
    float m_theta, m_phi, m_weightLink1, m_weightLink2;
};