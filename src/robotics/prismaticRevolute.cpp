#include "prismaticRevolute.h"
#include <assert.h>
#include <algorithm>


PrismaticRevolute::PrismaticRevolute(std::vector<std::shared_ptr<ArmComponent>> components,
    float weightLink1, float weightLink2)
{
    m_weightLink1 = weightLink1;
    m_weightLink2 = weightLink2;
    // TODO: replace next line with dynamic setter
    m_maxJoint1 = 80.f;

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

    m_delta = m_jointComponents[0]->getTranslation().x;
    m_phi = m_jointComponents[1]->getRotation().z;
}

void PrismaticRevolute::actuateJoints(std::vector<float> joints) 
{
    m_jointComponents[0]->setTranslation(Vector3d(joints[0], 0.f, 0.f));
    m_jointComponents[0]->setLocalTransform();
    m_jointComponents[1]->setRotation(Vector3d(0.f, 0.f, joints[1]));
    m_jointComponents[1]->setLocalTransform();

    for (auto component : m_components)
        component->setGlobalTransform();
}

std::vector<float> PrismaticRevolute::inverseKinematics(float x, float y) 
{
    Vector3d origin = Vector3d(0.f, 0.f), endEffectorVector = Vector3d(x, y), ip1 = Vector3d(), ip2 = Vector3d();

    Geometry::Circle c = Geometry::Circle(m_lengthLink2, endEffectorVector);
    c.getIntersectionPointsWithSegment(origin, Vector3d(0.f, 1000.f), ip1, ip2);

    Vector3d u = endEffectorVector - ip1;
    Vector3d v = endEffectorVector - ip2;

    float phi1 = Vector3d(0.f, 1.f).angleToVector(u) * 180.f / 3.14;
    float phi2 = Vector3d(0.f, 1.f).angleToVector(v) * 180.f / 3.14;

    if (endEffectorVector.x > 0)
    {
        phi1 = 360.f - phi1;
        phi2 = 360.f - phi2;
    }

    float delta1 = ip1.y - m_lengthLink1;
    float delta2 = ip2.y - m_lengthLink1;

    std::vector<float> values = { delta1, phi1, delta2, phi2 };

    return values;
}

std::vector<std::vector<float>> PrismaticRevolute::getDeltasBetweenPoses(float x, float y) 
{
    std::vector<float> values = inverseKinematics(x, y);

    float joint1Trans = m_jointComponents[0]->getTranslation().x;
    float joint2Rot = m_jointComponents[1]->getRotation().z;

    float dd1  = values[0] - joint1Trans;
    float dp1A = values[1] - joint2Rot;
    float dp1B = dp1A + (dp1A < 0.f ? 360.f : -361.f);
    float dd2  = values[2] - joint1Trans;
    float dp2A = values[3] - joint2Rot;
    float dp2B = dp2A + (dp2A < 0.f ? 360.f : -361.f);


    std::vector<std::vector<float>> deltas = {};

    if ((values[0] + m_lengthLink1) < m_lengthLink1 + m_maxJoint1)
    {
        deltas.push_back({ dd1, dp1A, std::abs(dd1) * m_weightLink1 + std::abs(dp1A) * m_weightLink2 });
        deltas.push_back({ dd1, dp1B, std::abs(dd1) * m_weightLink1 + std::abs(dp1B) * m_weightLink2 });
    }
    if ((values[2] + m_lengthLink1) < m_lengthLink1 + m_maxJoint1)
    {
        deltas.push_back({ dd2, dp2A, std::abs(dd2) * m_weightLink1 + std::abs(dp2A) * m_weightLink2 });
        deltas.push_back({ dd2, dp2B, std::abs(dd2) * m_weightLink1 + std::abs(dp2B) * m_weightLink2 });
    }
    std::sort(deltas.begin(), deltas.end(), compareDelta);

    return deltas;
}

std::vector<std::vector<Vector3d>> PrismaticRevolute::interpolate(float x, float y, int step) 
{
    std::vector<std::vector<Vector3d>> retData = { };

    std::vector<std::vector<float>> deltas = getDeltasBetweenPoses(x, y);

    float phi = m_jointComponents[0]->getTranslation().x;
    float delta = m_jointComponents[1]->getRotation().z;

    float deltaT = deltas[0][0], deltaD = deltas[0][1];
    float maxDelta = std::max(std::abs(deltaT), std::abs(deltaD));
    
    float unitDeltaP = deltaT / maxDelta;
    float unitDeltaD = deltaD / maxDelta;

    for (int i = 0; i < (int)maxDelta; i++)
    {
        if (i%step == 0)
        {
            std::vector<Vector3d> stepToRender = { Vector3d(0.f, 0.f, 0.f) };
            
            float joint1Trans = m_jointComponents[0]->getTranslation().x;
            float joint2Rot = m_jointComponents[1]->getRotation().z;
            float trans = joint1Trans + unitDeltaP;
            float rot = joint2Rot + unitDeltaD;
            
            if (rot < 0.f)
                rot += 360.f;
            if (rot > 360.f)
                rot = fmod(rot, 360.f);

            actuateJoints({ trans, rot });
            Vector3d joint2Pos = m_jointComponents[1]->getTransformedPoints()[0];
            Vector3d endEffPos = m_endEffComponent->getTransformedPoints()[0];

            stepToRender.push_back(Vector3d(joint2Pos.x, joint2Pos.y, 0.f));
            stepToRender.push_back(Vector3d(endEffPos.x, endEffPos.y, 0.f));
            stepToRender.push_back(Vector3d(trans, rot, 0.f));
            retData.push_back(stepToRender);
        }
        phi += unitDeltaP;
        delta += unitDeltaD;

        if (delta < 0) delta += 360.f;
        if (delta > 360.f)
            delta = fmod(delta, 360.f);
        std::vector<float> rotate = { phi, delta };
        
        actuateJoints(rotate);
    }
    std::vector<Vector3d> stepToRender = { Vector3d(0.f, 0.f, 0.f) };
    Vector3d joint2Pos = m_jointComponents[1]->getTransformedPoints()[0];
    Vector3d endEffPos = m_endEffComponent->getTransformedPoints()[0];

    stepToRender.push_back(Vector3d(joint2Pos.x, joint2Pos.y, 0.f));
    stepToRender.push_back(Vector3d(endEffPos.x, endEffPos.y, 0.f));
    stepToRender.push_back(Vector3d(phi, delta, 0.f));
    retData.push_back(stepToRender);

    return retData;
}

bool PrismaticRevolute::compareDelta(std::vector<float> a, std::vector<float> b) 
{
    return a[2] < b[2];
}