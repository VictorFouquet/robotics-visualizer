#include "revolutePrismatic.h"
#include <assert.h>
#include <algorithm>

#ifndef PI
#define PI 3.14159265
#endif

static float rad(float n)
{
    return 2 * PI * (n / 360);
}

RevolutePrismatic::RevolutePrismatic(float lenghtLink1, float lenghtLink2, float phi, float delta, float weightLink1, float weightLink2) 
    : m_delta(delta), m_phi(phi), m_weightLink1(weightLink1), m_weightLink2(weightLink2)
{   
    m_lengthLink1 = lenghtLink1;
    m_lengthLink2 = lenghtLink2;
    std::vector<float> links = { delta, phi };
    m_rotations = { delta, phi };
    actuateJoints(links);
}

void RevolutePrismatic::actuateJoints(std::vector<float> links) 
{
    m_rotations = links;
    Matrix m1 = Matrix::rotate(0.f, 0.f, links[0]) * Matrix::translate(m_lengthLink1, 0.f, 0.f);
    Matrix m2 = Matrix::rotate(0.f, 0.f, 0.f) * Matrix::translate(m_lengthLink2 + links[1], 0.f, 0.f);
    Matrix m3 = m1 * m2;

    Vector3d p1(0.f, 0.f, 0.f);

    m_endEffector = m3 * p1;

    m_transforms = { m1, m2, m3 };
    m_joints = { Vector3d(0.f, 0.f, 0.f), m1 * p1 };

    Vector3d p2(0.f, 0.f, 0.f);
    p2 = m1 * p1;
    std::pair<Vector3d, Vector3d> link1 = { p1, p2 };
    std::pair<Vector3d, Vector3d> link2 = { p2, m_endEffector };
    m_links = { link1, link2 };
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

    float dp1 = values[0] - m_rotations[0];
    float dp2 = values[0] - m_rotations[0] - 360.f;
    float dd  = values[1] - m_rotations[1];

    if (m_joints[1].y < 0)
    {
        dp1 = 360 - m_rotations[0] + values[0];
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

    float phi = m_rotations[0], delta = m_rotations[1];

    float deltaT = deltas[0][0], deltaD = deltas[0][1];
    float maxDelta = std::max(std::abs(deltaT), std::abs(deltaD));
    
    float unitDeltaP = deltaT / maxDelta;
    float unitDeltaD = deltaD / maxDelta;

    for (int i = 0; i < (int)maxDelta; i++)
    {
        if (i%step == 0)
        {
            std::vector<Vector3d> stepToRender = { Vector3d(0.f, 0.f, 0.f) };

            float roatA = m_rotations[0] + unitDeltaP;
            float roatB = m_rotations[1] + unitDeltaD;
            if (roatA < 0.f)
                roatA += 360.f;
            if (roatA > 360.f)
                roatA = fmod(roatA, 360.f);

            actuateJoints({ roatA, roatB });
            stepToRender.push_back(Vector3d(m_joints[1].x, m_joints[1].y, 0.f));
            stepToRender.push_back(Vector3d(m_endEffector.x, m_endEffector.y, 0.f));
            stepToRender.push_back(Vector3d(roatA, roatB, 0.f));
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
    stepToRender.push_back(Vector3d(m_joints[1].x, m_joints[1].y, 0.f));
    stepToRender.push_back(Vector3d(m_endEffector.x, m_endEffector.y, 0.f));
    stepToRender.push_back(Vector3d(phi, delta, 0.f));
    retData.push_back(stepToRender);

    return retData;
}

bool RevolutePrismatic::compareDelta(std::vector<float> a, std::vector<float> b) 
{
    return true;
}