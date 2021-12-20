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

    return {};
}

std::vector<std::vector<Vector3d>> PrismaticRevolute::interpolate(float x, float y, int step) 
{
    std::vector<std::vector<Vector3d>> retData = { };

    return retData;
}

bool PrismaticRevolute::compareDelta(std::vector<float> a, std::vector<float> b) 
{
    return a[2] < b[2];
}