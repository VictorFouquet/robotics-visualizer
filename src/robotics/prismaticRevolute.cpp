#include "prismaticRevolute.h"
#include <assert.h>
#include <algorithm>

#ifndef PI
#define PI 3.14159265
#endif

static float rad(float n)
{
    return 2 * PI * (n / 360);
}

PrismaticRevolute::PrismaticRevolute(float lenghtLink1, float lenghtLink2, float delta, float phi,
        float maxJoint1, float maxJoint2, float weightLink1, float weightLink2) 
    : m_delta(delta), m_phi(phi), m_weightLink1(weightLink1), m_weightLink2(weightLink2)
{
    m_maxJoint1 = maxJoint1;
    m_maxJoint2 = maxJoint2;
    m_lengthLink1 = lenghtLink1;
    m_lengthLink2 = lenghtLink2;
    std::vector<float> links = { delta, phi };
    m_rotations = { delta, phi };
    actuateJoints(links);
}

void PrismaticRevolute::actuateJoints(std::vector<float> links) 
{
    m_rotations = links;
    Matrix m1 = Matrix::rotate(0.f, 0.f, 90.f) * Matrix::translate(m_lengthLink1 + links[0], 0.f, 0.f);
    Matrix m2 = Matrix::rotate(0.f, 0.f, links[1]) * Matrix::translate(m_lengthLink2, 0.f, 0.f);
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

    float dd1  = values[0] - m_rotations[0];
    float dp1A = values[1] - m_rotations[1];
    float dp1B = values[1] - m_rotations[1] - 360.f;
    float dd2  = values[2] - m_rotations[0];
    float dp2A = values[3] - m_rotations[1];
    float dp2B = values[3] - m_rotations[1] - 360.f;

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
            if (roatB < 0.f)
                roatB += 360.f;
            if (roatB > 360.f)
                roatB = fmod(roatB, 360.f);

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

bool PrismaticRevolute::compareDelta(std::vector<float> a, std::vector<float> b) 
{
    return a[2] < b[2];
}