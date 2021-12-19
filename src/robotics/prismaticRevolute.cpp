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

}

std::vector<float> PrismaticRevolute::inverseKinematics(float x, float y) 
{
    return {};
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