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

RevoluteRevolute::RevoluteRevolute(float lenghtLink1, float lenghtLink2, float theta, float phi)
    : m_lenghtLink1(lenghtLink1), m_lenghtLink2(lenghtLink2), m_theta(theta), m_phi(phi)
{
    std::vector<float> links = { phi, theta };
    m_rotations = { theta, phi };
    rotateJoint(links);
}

void RevoluteRevolute::rotateJoint(std::vector<float> links) 
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

void RevoluteRevolute::rotateLink(int link, int theta) 
{
    
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

    std::vector<std::vector<float>> deltas{
        { dt1A, dp1A, std::abs(dt1A) + std::abs(dp1A) },
        { dt1A, dp1B, std::abs(dt1A) + std::abs(dp1B) },
        { dt1B, dp1A, std::abs(dt1B) + std::abs(dp1A) },
        { dt1B, dp1B, std::abs(dt1B) + std::abs(dp1B) },
        { dt2A, dp2A, std::abs(dt2A) + std::abs(dp2A) },
        { dt2A, dp2B, std::abs(dt2A) + std::abs(dp2B) },
        { dt2B, dp2A, std::abs(dt2B) + std::abs(dp2A) },
        { dt2B, dp2B, std::abs(dt2B) + std::abs(dp2B) },
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
    // -1, -70
    float maxDelta = std::max(std::abs(deltaT), std::abs(deltaP));
    
    float unitDeltaT = deltaT / maxDelta;
    float unitDeltaP = deltaP / maxDelta;

    for (int i = 0; i < (int)maxDelta; i++)
    {
        if (i%step == 0)
        {
            std::vector<Vector3d> stepToRender = { Vector3d(0.f, 0.f, 0.f) };

            // rotateJoint({ values[0], values[1]});
            rotateJoint({ m_rotations[0] + unitDeltaT, m_rotations[1] + unitDeltaP });
            // std::vector<float> values = inverseKinematics(m_endEffector.x, m_endEffector.y);
            stepToRender.push_back(Vector3d(m_joints[1].x, m_joints[1].y, 0.f));
            stepToRender.push_back(Vector3d(m_endEffector.x, m_endEffector.y, 0.f));
            stepToRender.push_back(Vector3d(m_rotations[0], m_rotations[1], 0.f));
            retData.push_back(stepToRender);
        }
        theta += unitDeltaT;
        phi += unitDeltaP;

        if (phi < 0) phi = 360.f - phi;
        std::vector<float> rotate = { theta, phi };
        
        rotateJoint(rotate);
    }
    std::vector<Vector3d> stepToRender = { Vector3d(0.f, 0.f, 0.f) };
    stepToRender.push_back(Vector3d(m_joints[1].x, m_joints[1].y, 0.f));
    stepToRender.push_back(Vector3d(m_endEffector.x, m_endEffector.y, 0.f));
    stepToRender.push_back(Vector3d(theta, phi, 0.f));
    retData.push_back(stepToRender);

    return retData;
}

void RobotArm::forwardKinematic2DOF_DEMO()
{
    //---------------------------------------------
    // Simulating a 2 degrees of freedom robot arm
    //---------------------------------------------
    // The robot arm is composed of 2 links and 2 rotational joints.
    // First link is attached to the ground by a joint, second link is attached to 
    // the first link by another joint.
    // End effector is considered to be the end of the second link.
    // Position of a link can be represented by matrix.
    // Location of the end effector can be computed by multiplying those matrices.

    std::cout << "Simulating two DOF robot arm: " << std::endl;

    Matrix m1 = Matrix::translate(10.f, 0.f, 0.f);
    Matrix m2 = Matrix::translate(10.f, 0.f, 0.f);
    Matrix m3 = m1 * m2;

    Vector3d p(0.f, 0.f, 0.f);

    std::cout << "Position of second joint when no rotation is applied : ";
    (m1 * p).print();
    std::cout << "Position of end effector when no rotation is applied : ";
    (m3 * p).print();

    m1 = Matrix::rotate(0.f, 0.f, 90.f) * Matrix::translate(10.f, 0.f, 0.f);
    m3 = m1 * m2;

    std::cout << "\nPosition of second joint when a rotation of 90° is applied on first joint : ";
    (m1 * p).print();
    std::cout << "Position of end effector when a rotation of 90° is applied on first joint : ";
    (m3 * p).print();

    m2 = Matrix::rotate(0.f, 0.f, 90.f) * Matrix::translate(10.f, 0.f, 0.f);
    m3 = m1 * m2;

    std::cout << "\nPosition of second joint when a rotation of 90° is applied on both joints : ";
    (m1 * p).print();
    std::cout << "Position of end effector when a rotation of 90° is applied on both joints : ";
    (m3 * p).print();
}


void RobotArm::forwardKinematics6DOF(float theta1, float theta2, float theta3, float theta4, float theta5, float theta6)
{
    // Reference robot-arm illustration found here :
    // https://www.semanticscholar.org/paper/Multi-DOF-counterbalance-mechanism-for-low-cost%2C-Kim-Min/fe6beac6456dacbd54fe7b9f149c26f42fcc4fb8/figure/5
    
    Vector3d endEffector(0.f, 0.f, 0.f);

    Matrix joint1 = Matrix::rotate(0.f, 0.f, theta1) * Matrix::translate(0.f, 0.f, 10.f);
    std::cout << "End position of link 1 :\n";
    (joint1 * endEffector).print();

    Matrix joint2 = Matrix::rotate(90.f, 0.f, theta2) * Matrix::translate(10.f, 0.f, 0.f);
    std::cout << "End position of link 2 :\n";
    ((joint1 * joint2) * endEffector).print();

    Matrix joint3 = Matrix::rotate(0.f, 0.f, theta3) * Matrix::translate(10.f, 0.f, 0.f);
    std::cout << "End position of link 3 :\n";
    ((joint1 * joint2 * joint3) * endEffector).print();

    Matrix joint4 = Matrix::rotate(0.f, 0.f, theta4) * Matrix::translate(10.f, 0.f, 0.f);
    std::cout << "End position of link 4 :\n";
    ((joint1 * joint2 * joint3 * joint4) * endEffector).print();

    Matrix joint5 = Matrix::rotate(-90.f, 0.f, theta5) * Matrix::translate(10.f, 0.f, 0.f);
    std::cout << "End position of link 5 :\n";
    ((joint1 * joint2 * joint3 * joint4 * joint5) * endEffector).print();

    Matrix joint6 = Matrix::rotate(0.f, 90.f, theta6) * Matrix::translate(0.f, 0.f, 10.f);
    Matrix transform = joint1 * joint2 * joint3 * joint4 * joint5 * joint6;

    std::cout << "Position of end effector :\n";
    (transform * endEffector).print();
}
