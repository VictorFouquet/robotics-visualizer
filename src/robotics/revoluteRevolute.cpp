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
    
    std::vector<Vector3d> points = m_endEffComponent->getTransformedPoints();
    m_endEffectorLength = points.back().x - points[0].x;

    m_phi = m_jointComponents[0]->getRotation().z;
    m_theta = m_jointComponents[1]->getRotation().z;
}

void RevoluteRevolute::actuateJoints(std::vector<float> links) 
{
    m_jointComponents[0]->setRotation(Vector3d(0.f, 0.f, links[0]));
    m_jointComponents[0]->setLocalTransform();
    m_jointComponents[1]->setRotation(Vector3d(0.f, 0.f, links[1]));
    m_jointComponents[1]->setLocalTransform();
    if (links.size() > 2)
    {
        m_jointComponents[2]->setRotation(Vector3d(0.f, 0.f, links[2]));
        m_jointComponents[2]->setLocalTransform();
    }

    for (auto component : m_components)
        component->setGlobalTransform();
}

std::vector<std::vector<float>> RevoluteRevolute::inverseKinematics(float x, float y, float rotz) 
{
    Matrix m = Matrix::translate(x, y, 0.f) * Matrix::rotate(0.f, 0.f, rotz) *  Matrix::identity();

    float rot0Z = rotz, rot1Z = rotz + 90.f, rot2Z = rotz + 180.f, rot3Z = rotz + 270.f;
    if (rot0Z > 360.f) rot0Z = fmod(rot0Z, 360.f);
    if (rot1Z > 360.f) rot1Z = fmod(rot1Z, 360.f);
    if (rot2Z > 360.f) rot2Z = fmod(rot2Z, 360.f);
    if (rot3Z > 360.f) rot3Z = fmod(rot3Z, 360.f);

    std::vector<std::vector<Vector3d>> poses = {
        { m * Vector3d(-m_endEffectorLength, 0.f), Vector3d(0.f, 0.f, rot0Z) },
        { m * Vector3d(0.f, -m_endEffectorLength), Vector3d(0.f, 0.f, rot1Z) },
        { m * Vector3d(m_endEffectorLength, 0.f),  Vector3d(0.f, 0.f, rot2Z) },
        { m * Vector3d(0.f, m_endEffectorLength),  Vector3d(0.f, 0.f, rot3Z) }
    };

    std::vector<std::vector<float>> values = {};
    for (auto pose : poses)
    {
        int pMag = pose[0].magnitude();
        int maxL = m_lengthLink1 + m_lengthLink2;
        if (pMag < maxL)
        {
            Vector3d endEffectorVector = Vector3d(pose[0].x, pose[0].y);
            Vector3d u1 = Vector3d(), u2 = Vector3d();

            Geometry::Circle c1(m_lengthLink1, Vector3d(0.f, 0.f));
            Geometry::Circle c2(m_lengthLink2, Vector3d(pose[0].x, pose[0].y));

            c1.getIntersectionPointsWithCircle(c2, u1, u2);

            if (u1.x + u2.x + u1.y + u2.y == 0)
                continue;

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

            Vector3d w = Vector3d(x, y) - pose[0];

            float gamma1 = w.angleToVector(v1) * 180.f / 3.14;
            float gamma2 = w.angleToVector(v2) * 180.f / 3.14;
            Vector3d v3 = v1 + w;
            Vector3d v4 = v2 + w;

            int or1 = Vector3d().orientation(v1, v3);
            int or2 = Vector3d().orientation(v2, v4);

            if (or1 > 0)
                gamma1 = 360.f - gamma1;
            if (or2 > 0)
                gamma2 = 360.f - gamma2;

            values.push_back({ 
                theta1, phi1, gamma1,
                theta2, phi2, gamma2
            });
        }
    }

    return values;
}

bool RevoluteRevolute::compareDelta(std::vector<float> a, std::vector<float> b)
{
    return a.back() < b.back();
}

std::vector<std::vector<float>> RevoluteRevolute::getDeltasBetweenPoses(float x, float y, float rotz) 
{
    std::vector<std::vector<float>> deltas = {};

    std::vector<std::vector<float>> vals = inverseKinematics(x, y, rotz);
    for (auto values : vals)
    {
        float theta1 = values[0];
        float phi1   = values[1];
        float gamma1 = values[2]; 
        float theta2 = values[3];
        float phi2   = values[4];
        float gamma2 = values[5];

        float joint1Rot = m_jointComponents[0]->getRotation().z;
        Vector3d joint1Pos = m_jointComponents[0]->getTransformedPoints()[0];
        float joint2Rot = m_jointComponents[1]->getRotation().z;
        Vector3d joint2Pos = m_jointComponents[1]->getTransformedPoints()[0];
        float joint3Rot = m_jointComponents[2]->getRotation().z;
        Vector3d joint3Pos = m_jointComponents[2]->getTransformedPoints()[0];
        Vector3d endEffectorPos = m_endEffComponent->getTransformedPoints()[0];

        float dt1A = theta1 - joint1Rot;
        float dt1B = theta1 - joint1Rot - 360.f;
        float dt2A = theta2 - joint1Rot;
        float dt2B = theta2 - joint1Rot - 360.f;
        float dp1A = phi1   - joint2Rot;
        float dp1B = phi1   - joint2Rot - 360.f;
        float dp2A = phi2   - joint2Rot;
        float dp2B = phi2   - joint2Rot - 360.f;
        float dg1A = gamma1 - joint3Rot;
        float dg1B = gamma1 - joint3Rot - 360.f;
        float dg2A = gamma2 - joint3Rot;
        float dg2B = gamma2 - joint3Rot - 360.f;

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

        if (joint2Pos.orientation(endEffectorPos, Vector3d(x, y)) > 0)
        {
            dg1A = 360 - joint3Rot + gamma1;
            dg1B = -360.f + dg1A;
            dg2A = 360.f - joint3Rot + gamma2;
            dg2B = -360.f + dg2A;
        }

        deltas.push_back( { dt1A, dp1A, dg1A, std::abs(dt1A) * m_weightLink1 + std::abs(dp1A) * m_weightLink2 + std::abs(dg1A) } );
        deltas.push_back( { dt1A, dp1A, dg1B, std::abs(dt1A) * m_weightLink1 + std::abs(dp1A) * m_weightLink2 + std::abs(dg1B) } );
        deltas.push_back( { dt1A, dp1B, dg1A, std::abs(dt1A) * m_weightLink1 + std::abs(dp1B) * m_weightLink2 + std::abs(dg1A) } );
        deltas.push_back( { dt1A, dp1B, dg1B, std::abs(dt1A) * m_weightLink1 + std::abs(dp1B) * m_weightLink2 + std::abs(dg1B) } );

        deltas.push_back( { dt1B, dp1A, dg1A, std::abs(dt1B) * m_weightLink1 + std::abs(dp1A) * m_weightLink2 + std::abs(dg1A) } );
        deltas.push_back( { dt1B, dp1A, dg1B, std::abs(dt1B) * m_weightLink1 + std::abs(dp1A) * m_weightLink2 + std::abs(dg1B) } );
        deltas.push_back( { dt1B, dp1B, dg1A, std::abs(dt1B) * m_weightLink1 + std::abs(dp1B) * m_weightLink2 + std::abs(dg1A) } );
        deltas.push_back( { dt1B, dp1B, dg1B, std::abs(dt1B) * m_weightLink1 + std::abs(dp1B) * m_weightLink2 + std::abs(dg1B) } );

        deltas.push_back( { dt2A, dp2A, dg2A, std::abs(dt2A) * m_weightLink1 + std::abs(dp2A) * m_weightLink2 + std::abs(dg2A) } );
        deltas.push_back( { dt2A, dp2A, dg2B, std::abs(dt2A) * m_weightLink1 + std::abs(dp2A) * m_weightLink2 + std::abs(dg2B) } );
        deltas.push_back( { dt2A, dp2B, dg2A, std::abs(dt2A) * m_weightLink1 + std::abs(dp2B) * m_weightLink2 + std::abs(dg2A) } );
        deltas.push_back( { dt2A, dp2B, dg2B, std::abs(dt2A) * m_weightLink1 + std::abs(dp2B) * m_weightLink2 + std::abs(dg2B) } );

        deltas.push_back( { dt2B, dp2A, dg2A, std::abs(dt2B) * m_weightLink1 + std::abs(dp2A) * m_weightLink2 + std::abs(dg2A) } );
        deltas.push_back( { dt2B, dp2A, dg2B, std::abs(dt2B) * m_weightLink1 + std::abs(dp2A) * m_weightLink2 + std::abs(dg2B) } );
        deltas.push_back( { dt2B, dp2B, dg2A, std::abs(dt2B) * m_weightLink1 + std::abs(dp2B) * m_weightLink2 + std::abs(dg2A) } );
        deltas.push_back( { dt2B, dp2B, dg2B, std::abs(dt2B) * m_weightLink1 + std::abs(dp2B) * m_weightLink2 + std::abs(dg2B) } );
    }

    std::sort(deltas.begin(), deltas.end(), compareDelta);

    return deltas;
}

std::vector<std::vector<Vector3d>> RevoluteRevolute::interpolate(Vector3d pos, float rot, int step) 
{
    std::vector<std::vector<float>> deltas = getDeltasBetweenPoses(pos.x, pos.y, rot);
    std::vector<std::vector<Vector3d>> retData = { };
    
    float theta = m_jointComponents[0]->getRotation().z;
    float phi = m_jointComponents[1]->getRotation().z;
    float gamma = m_jointComponents[2]->getRotation().z;

    float joint1Rot = m_jointComponents[0]->getRotation().z;
    float joint2Rot = m_jointComponents[1]->getRotation().z;
    float joint3Rot = m_jointComponents[2]->getRotation().z;

    float deltaT = deltas[0][0], deltaP = deltas[0][1], deltaG = deltas[0][2];
    float maxDelta = std::max(std::abs(deltaT), std::abs(deltaP));
    maxDelta = std::max(maxDelta, std::abs(deltaG));
    
    float unitDeltaT = deltaT / maxDelta;
    float unitDeltaP = deltaP / maxDelta;
    float unitDeltaG = deltaG / maxDelta;

    for (int i = 0; i < (int)maxDelta; i++)
    {
        if (i%step == 0)
        {
            std::vector<Vector3d> stepToRender = { Vector3d(0.f, 0.f, 0.f) };
            joint1Rot = m_jointComponents[0]->getRotation().z;
            joint2Rot = m_jointComponents[1]->getRotation().z;
            joint3Rot = m_jointComponents[2]->getRotation().z;

            float rotA = joint1Rot + unitDeltaT;
            float rotB = joint2Rot + unitDeltaP;
            float rotC = joint3Rot + unitDeltaG;

            if (rotA < 0.f)
                rotA += 360.f;
            if (rotB < 0.f)
                rotB += 360.f;
            if (rotC < 0.f)
                rotC += 360.f;
            if (rotA > 360.f)
                rotA = fmod(rotA, 360.f);
            if (rotB < 0.f)
                rotB = fmod(rotB, 360.f);
            if (rotC < 0.f)
                rotC = fmod(rotC, 360.f);
            
            actuateJoints({ rotA, rotB, rotC });

            Vector3d joint2Pos = m_jointComponents[1]->getTransformedPoints()[0];
            std::vector<Vector3d> endEffPos = m_endEffComponent->getTransformedPoints();

            stepToRender.push_back(Vector3d(joint2Pos.x, joint2Pos.y, 0.f));
            for (auto p : endEffPos)
                stepToRender.push_back(p);
            stepToRender.push_back(Vector3d(rotA, rotB, 0.f));
            retData.push_back(stepToRender);
        }
        theta += unitDeltaT;
        phi += unitDeltaP;
        gamma += unitDeltaG;

        if (phi < 0) phi += 360.f;
        if (theta < 0) theta += 360.f;
        if (gamma < 0) gamma += 360.f;

        if (theta > 360.f)
            theta = fmod(theta, 360.f);
        if (phi > 360.f)
            phi = fmod(phi, 360.f);
        if (gamma > 360.f)
            gamma = fmod(gamma, 360.f);

        std::vector<float> rotate = { theta, phi, gamma };
        
        actuateJoints(rotate);
    }

    std::vector<Vector3d> stepToRender = { Vector3d(0.f, 0.f, 0.f) };
    Vector3d joint2Pos = m_jointComponents[1]->getTransformedPoints()[0];
    std::vector<Vector3d> endEffPos = m_endEffComponent->getTransformedPoints();

    stepToRender.push_back(Vector3d(joint2Pos.x, joint2Pos.y, 0.f));
    for (auto p : endEffPos)
        stepToRender.push_back(p);
    stepToRender.push_back(Vector3d(theta, phi, 0.f));
    retData.push_back(stepToRender);

    return retData;
}
