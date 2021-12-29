#include "revoluteRevolute.h"
#include "circle.h"
#include <assert.h>
#include <algorithm>


RevoluteRevolute::RevoluteRevolute(std::vector<std::shared_ptr<ArmComponent>> components, 
    float weightLink1, float weightLink2)
{
    m_weightLink1 = weightLink1;
    m_weightLink2 = weightLink2;
    m_components = components;

    for (auto component : m_components)
    {
        if (component->isType(ArmComponentType::rigidBody))
            m_rigidBodies.push_back(component);
        if (component->isType(ArmComponentType::joint))
            m_jointComponents.push_back(component);
        if (component->isType(ArmComponentType::endEffector))
            m_endEffComponent = component;
        component->setGlobalTransform();
    }

    m_lengthLink1 = m_rigidBodies[0]->getTranslation().x;
    m_lengthLink2 = m_rigidBodies[1]->getTranslation().x;

    m_phi = m_jointComponents[0]->getRotation().z;
    m_theta = m_jointComponents[1]->getRotation().z;
}

void RevoluteRevolute::actuateJoints(std::vector<float> links) 
{
    m_jointComponents[0]->setRotation(Vector3d(0.f, 0.f, links[0]));
    m_jointComponents[0]->setLocalTransform();
    m_jointComponents[1]->setRotation(Vector3d(0.f, 0.f, links[1]));
    m_jointComponents[1]->setLocalTransform();

    for (auto component : m_components)
        component->setGlobalTransform();
}

std::vector<float> RevoluteRevolute::inverseKinematics(float x, float y) 
{
    Vector3d endEffectorVector = Vector3d(x, y);
    Vector3d u1 = Vector3d(), u2 = Vector3d();

    Geometry::Circle c1(m_lengthLink1, Vector3d(0.f, 0.f));
    Geometry::Circle c2(m_lengthLink2, Vector3d(x, y));

    c1.getIntersectionPointsWithCircle(c2, u1, u2);

    Vector3d v1 = endEffectorVector - u1;
    Vector3d v2 = endEffectorVector - u2;

    float theta1 = Vector3d(1.f, 0.f).angleToVector(u1) * 180.f / 3.14;
    float theta2 = Vector3d(1.f, 0.f).angleToVector(u2) * 180.f / 3.14;
    if (u1.y < 0)
    {
        theta1 = 360.f - theta1;
    }
    if (u2.y < 0)
    {
        theta2 = 360.f - theta2;
    }
    float phi1 = v1.angleToVector(u1) * 180.f / 3.14;
    float phi2 = 360.f - phi1;

    std::vector<float> values = { 
        theta1, phi1,
        theta2, phi2
    };

    return values;
}

bool RevoluteRevolute::compareDelta(std::vector<float> a, std::vector<float> b)
{
    return a[2] < b[2];
}

std::vector<std::vector<float>> RevoluteRevolute::getDeltasBetweenPoses(float x, float y) 
{ 
    std::vector<float> values = inverseKinematics(x, y);
    float theta1 = values[0];
    float phi1 = values[1];
    float theta2 = values[2];
    float phi2 = values[3];

    float joint1Rot = m_jointComponents[0]->getRotation().z;
    Vector3d joint1Pos = m_jointComponents[0]->getTransformedPoints()[0];
    float joint2Rot = m_jointComponents[1]->getRotation().z;
    Vector3d joint2Pos = m_jointComponents[1]->getTransformedPoints()[0];
    Vector3d endEffectorPos = m_endEffComponent->getTransformedPoints()[0];

    float dt1A = theta1 - joint1Rot;
    float dt1B = theta1 - joint1Rot - 360.f;
    float dt2A = theta2 - joint1Rot;
    float dt2B = theta2 - joint1Rot - 360.f;
    float dp1A = phi1   - joint2Rot;
    float dp1B = phi1   - joint2Rot - 360.f;
    float dp2A = phi2   - joint2Rot;
    float dp2B = phi2   - joint2Rot - 360.f;

    if (joint2Pos.y < 0)
    {
        dt1A = 360 - joint1Rot + theta1;
        dt1B = -360.f + dt1A;
        dt2A = 360.f - joint1Rot + theta2;
        dt2B = -360.f + dt2A;
    }
    int orientation = joint1Pos.orientation(joint2Pos, endEffectorPos);
    if (joint1Pos.orientation(joint2Pos, endEffectorPos) > 0)
    {
        dp1A = 360 - joint2Rot + phi1;
        dp1B = -360.f + dp1A;
        dp2A = 360.f - joint2Rot + phi2;
        dp2B = -360.f + dp2A;
    }

    std::vector<std::vector<float>> deltas{
        { dt1A, dp1A, std::abs(dt1A) * m_weightLink1 + std::abs(dp1A) * m_weightLink2 },
        { dt1A, dp1B, std::abs(dt1A) * m_weightLink1 + std::abs(dp1B) * m_weightLink2 },
        { dt1B, dp1A, std::abs(dt1B) * m_weightLink1 + std::abs(dp1A) * m_weightLink2 },
        { dt1B, dp1B, std::abs(dt1B) * m_weightLink1 + std::abs(dp1B) * m_weightLink2 },
        { dt2A, dp2A, std::abs(dt2A) * m_weightLink1 + std::abs(dp2A) * m_weightLink2 },
        { dt2A, dp2B, std::abs(dt2A) * m_weightLink1 + std::abs(dp2B) * m_weightLink2 },
        { dt2B, dp2A, std::abs(dt2B) * m_weightLink1 + std::abs(dp2A) * m_weightLink2 },
        { dt2B, dp2B, std::abs(dt2B) * m_weightLink1 + std::abs(dp2B) * m_weightLink2 },
    };
    std::sort(deltas.begin(), deltas.end(), compareDelta);

    return deltas;
}

std::vector<std::vector<Vector3d>> RevoluteRevolute::interpolate(float x, float y, int step) 
{
    std::vector<std::vector<Vector3d>> retData = { };
    
    float theta = m_jointComponents[0]->getRotation().z, phi = m_jointComponents[1]->getRotation().z;
    float joint1Rot = m_jointComponents[0]->getRotation().z;
    float joint2Rot = m_jointComponents[1]->getRotation().z;

    std::vector<std::vector<float>> deltas = getDeltasBetweenPoses(x, y);
    float deltaT = deltas[0][0], deltaP = deltas[0][1];
    float maxDelta = std::max(std::abs(deltaT), std::abs(deltaP));
    
    float unitDeltaT = deltaT / maxDelta;
    float unitDeltaP = deltaP / maxDelta;

    for (int i = 0; i < (int)maxDelta; i++)
    {
        if (i%step == 0)
        {
            std::vector<Vector3d> stepToRender = { Vector3d(0.f, 0.f, 0.f) };
            joint1Rot = m_jointComponents[0]->getRotation().z;
            joint2Rot = m_jointComponents[1]->getRotation().z;
            float rotA = joint1Rot + unitDeltaT;
            float rotB = joint2Rot + unitDeltaP;
            if (rotA < 0.f)
                rotA += 360.f;
            if (rotB < 0.f)
                rotB += 360.f;
            if (rotA > 360.f)
                rotA = fmod(rotA, 360.f);
            if (rotB < 0.f)
                rotB = fmod(rotB, 360.f);
            
            actuateJoints({ rotA, rotB });
            Vector3d joint2Pos = m_jointComponents[1]->getTransformedPoints()[0];
            Vector3d endEffPos = m_endEffComponent->getTransformedPoints()[0];

            stepToRender.push_back(Vector3d(joint2Pos.x, joint2Pos.y, 0.f));
            stepToRender.push_back(Vector3d(endEffPos.x, endEffPos.y, 0.f));
            stepToRender.push_back(Vector3d(rotA, rotB, 0.f));
            retData.push_back(stepToRender);
        }
        theta += unitDeltaT;
        phi += unitDeltaP;

        if (phi < 0) phi += 360.f;
        if (theta < 0) theta += 360.f;
        if (theta > 360.f)
            theta = fmod(theta, 360.f);
        if (phi > 360.f)
            phi = fmod(phi, 360.f);
        std::vector<float> rotate = { theta, phi };
        
        actuateJoints(rotate);
    }
    std::vector<Vector3d> stepToRender = { Vector3d(0.f, 0.f, 0.f) };
    Vector3d joint2Pos = m_jointComponents[1]->getTransformedPoints()[0];
    Vector3d endEffPos = m_endEffComponent->getTransformedPoints()[0];

    stepToRender.push_back(Vector3d(joint2Pos.x, joint2Pos.y, 0.f));
    stepToRender.push_back(Vector3d(endEffPos.x, endEffPos.y, 0.f));
    stepToRender.push_back(Vector3d(theta, phi, 0.f));
    retData.push_back(stepToRender);

    return retData;
}
