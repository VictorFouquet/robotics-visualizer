#include "revolutePrismatic.h"
#include <assert.h>
#include <algorithm>


RevolutePrismatic::RevolutePrismatic(
    std::vector<std::shared_ptr<ArmComponent>> components,
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
    m_delta = m_jointComponents[1]->getTranslation().x;
}

void RevolutePrismatic::actuateJoints(std::vector<float> joints) 
{
    m_jointComponents[0]->setRotation(Vector3d(0.f, 0.f, joints[0]));
    m_jointComponents[0]->setLocalTransform();
    m_jointComponents[1]->setTranslation(Vector3d(joints[1], 0.f, 0.f));
    m_jointComponents[1]->setLocalTransform();

    for (auto component : m_components)
        component->setGlobalTransform();
}

std::vector<float> RevolutePrismatic::inverseKinematics(float x, float y) 
{
    Vector3d endEffectorVector = Vector3d(x, y);

    float phi = Vector3d(1.f, 0.f).angleToVector(endEffectorVector) * 180.f / 3.14;

    if (endEffectorVector.y < 0)
    {
        phi = 360.f - phi;
    }

    float delta = endEffectorVector.magnitude() - m_lengthLink1 - m_lengthLink2;

    std::vector<float> values = { phi, delta };

    return values;
}

std::vector<std::vector<float>> RevolutePrismatic::getDeltasBetweenPoses(float x, float y) 
{
    std::vector<float> values = inverseKinematics(x, y);
    float joint1Rot = m_jointComponents[0]->getRotation().z;
    float joint2Trans = m_jointComponents[1]->getTranslation().x;
    float joint2PosY = m_jointComponents[1]->getTransformedPoints()[0].y;

    float dp1 = values[0] - joint1Rot;
    float dp2 = values[0] - joint1Rot - 360.f;
    float dd  = values[1] - joint2Trans;

    if (joint2PosY < 0)
    {
        dp1 = 360 - joint1Rot + values[0];
        dp2 = -360.f + dp1;
    }

    std::vector<std::vector<float>> deltas = {
        { dp1, dd },
        { dp2, dd },
    };

    if (std::abs(deltas[0][0]) > std::abs(deltas[1][0]))
        std::swap(deltas[0], deltas[1]);

    return deltas;
}

std::vector<std::vector<Vector3d>> RevolutePrismatic::interpolate(float x, float y, int step) 
{
    std::vector<std::vector<Vector3d>> retData = { };

    std::vector<std::vector<float>> deltas = getDeltasBetweenPoses(x, y);

    float phi = m_jointComponents[0]->getRotation().z;
    float delta = m_jointComponents[1]->getTranslation().x;

    float deltaT = deltas[0][0], deltaD = deltas[0][1];
    float maxDelta = std::max(std::abs(deltaT), std::abs(deltaD));
    
    float unitDeltaP = deltaT / maxDelta;
    float unitDeltaD = deltaD / maxDelta;

    for (int i = 0; i < (int)maxDelta; i++)
    {
        if (i%step == 0)
        {
            std::vector<Vector3d> stepToRender = { Vector3d(0.f, 0.f, 0.f) };
            float joint1Rot = m_jointComponents[0]->getRotation().z;
            float joint2Trans = m_jointComponents[1]->getTranslation().x;
            float rot = joint1Rot + unitDeltaP;
            float trans = joint2Trans + unitDeltaD;
            if (rot < 0.f)
                rot += 360.f;
            if (rot > 360.f)
                rot = fmod(rot, 360.f);

            actuateJoints({ rot, trans });
            Vector3d joint2Pos = m_jointComponents[1]->getTransformedPoints()[0];
            Vector3d endEffPos = m_endEffComponent->getTransformedPoints()[0];

            stepToRender.push_back(Vector3d(joint2Pos.x, joint2Pos.y, 0.f));
            stepToRender.push_back(Vector3d(endEffPos.x, endEffPos.y, 0.f));
            stepToRender.push_back(Vector3d(rot, trans, 0.f));
            retData.push_back(stepToRender);
        }
        phi += unitDeltaP;
        delta += unitDeltaD;

        if (phi < 0) phi += 360.f;
        if (phi > 360.f)
            phi = fmod(phi, 360.f);
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

bool RevolutePrismatic::compareDelta(std::vector<float> a, std::vector<float> b) 
{
    return true;
}