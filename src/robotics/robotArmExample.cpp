#include "robotArmExample.h"
#include "circle.h"
#include <assert.h>
#include <algorithm>

#ifndef PI
#define PI 3.14159265
#endif

static float rad(float n)
{
    return 2 * PI * (n / 360);
}

RevoluteRevolute::RevoluteRevolute(float lenghtLink1, float lenghtLink2, float theta, float phi, float weightLink1, float weightLink2)
    : m_lenghtLink1(lenghtLink1), m_lenghtLink2(lenghtLink2), m_theta(theta), m_phi(phi), m_weightLink1(weightLink1), m_weightLink2(weightLink2)
{
    std::vector<float> links = { phi, theta };
    m_rotations = { theta, phi };
    actuateJoints(links);
}

void RevoluteRevolute::actuateJoints(std::vector<float> links) 
{
    m_rotations = links;
    Matrix m1 = Matrix::rotate(0.f, 0.f, links[0]) * Matrix::translate(m_lenghtLink1, 0.f, 0.f);
    Matrix m2 = Matrix::rotate(0.f, 0.f, links[1]) * Matrix::translate(m_lenghtLink2, 0.f, 0.f);
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

std::vector<float> RevoluteRevolute::inverseKinematics(float x, float y) 
{
    Vector3d endEffectorVector = Vector3d(x, y);
    Vector3d u1 = Vector3d(), u2 = Vector3d();

    Geometry::Circle c1(m_lenghtLink1, Vector3d(0.f, 0.f));
    Geometry::Circle c2(m_lenghtLink2, Vector3d(x, y));

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
        theta2, phi2,
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


    float dt1A = theta1 - m_rotations[0];
    float dt1B = theta1 - m_rotations[0] - 360.f;
    float dt2A = theta2 - m_rotations[0];
    float dt2B = theta2 - m_rotations[0] - 360.f;
    float dp1A = phi1   - m_rotations[1];
    float dp1B = phi1   - m_rotations[1] - 360.f;
    float dp2A = phi2   - m_rotations[1];
    float dp2B = phi2   - m_rotations[1] - 360.f;

    if (m_joints[1].y < 0)
    {
        dt1A = 360 - m_rotations[0] + theta1;
        dt1B = -360.f + dt1A;
        dt2A = 360.f - m_rotations[0] + theta2;
        dt2B = -360.f + dt2A;
    }
    int orientation = m_joints[0].orientation(m_joints[1], m_endEffector);
    if (m_joints[0].orientation(m_joints[1], m_endEffector) > 0)
    {
        dp1A = 360 - m_rotations[1] + phi1;
        dp1B = -360.f + dp1A;
        dp2A = 360.f - m_rotations[1] + phi2;
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
    
    float theta = m_rotations[0], phi = m_rotations[1];

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

            float roatA = m_rotations[0] + unitDeltaT;
            float roatB = m_rotations[1] + unitDeltaP;
            if (roatA < 0.f)
                roatA += 360.f;
            if (roatB < 0.f)
                roatB += 360.f;
            if (roatA > 360.f)
                roatA = fmod(roatA, 360.f);
            if (roatB < 0.f)
                roatB = fmod(roatB, 360.f);
            actuateJoints({ roatA, roatB });
            stepToRender.push_back(Vector3d(m_joints[1].x, m_joints[1].y, 0.f));
            stepToRender.push_back(Vector3d(m_endEffector.x, m_endEffector.y, 0.f));
            stepToRender.push_back(Vector3d(roatA, roatB, 0.f));
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
    stepToRender.push_back(Vector3d(m_joints[1].x, m_joints[1].y, 0.f));
    stepToRender.push_back(Vector3d(m_endEffector.x, m_endEffector.y, 0.f));
    stepToRender.push_back(Vector3d(theta, phi, 0.f));
    retData.push_back(stepToRender);

    return retData;
}



RevolutePrismatic::RevolutePrismatic(float lenghtLink1, float lenghtLink2, float phi, float delta, float weightLink1, float weightLink2) 
    : m_lenghtLink1(lenghtLink1), m_lenghtLink2(lenghtLink2), m_delta(delta), m_phi(phi), m_weightLink1(weightLink1), m_weightLink2(weightLink2)
{    
    std::vector<float> links = { delta, phi };
    m_rotations = { delta, phi };
    actuateJoints(links);
}

void RevolutePrismatic::actuateJoints(std::vector<float>) 
{
}

std::vector<float> RevolutePrismatic::inverseKinematics(float x, float y) 
{
    return {};
}

std::vector<std::vector<float>> RevolutePrismatic::getDeltasBetweenPoses(float x, float y) 
{
    return {};
}

std::vector<std::vector<Vector3d>> RevolutePrismatic::interpolate(float x, float y, int step) 
{
    return {};
}

bool RevolutePrismatic::compareDelta(std::vector<float> a, std::vector<float> b) 
{
    return true;
}