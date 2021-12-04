#include "robotArmExample.h"
#include <assert.h>
#include <algorithm>

RevoluteRevolute::RevoluteRevolute(float lenghtLink1, float lenghtLink2, float theta, float phi)
    : m_lenghtLink1(lenghtLink1), m_lenghtLink2(lenghtLink2), m_theta(theta), m_phi(phi)
{
    std::vector<float> links = { phi, theta };
    m_rotations = { theta, phi };
    rotateJoint(links);
}

void RevoluteRevolute::rotateJoint(std::vector<float> links) 
{
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
    float a1 = m_lenghtLink1, a2 = m_lenghtLink2;
    float r1 = std::sqrt(std::pow(x, 2) + std::pow(y, 2));

    float phiRad = computeAngleFromLength(a1, a2, r1);

    float phi1 = 180.f - phiRad * 180.f / 3.14;
    float phi2 = 360 - phi1; 
    
    float length = computeLengthFromAngleAndHypotenuse(a2, phiRad);

    float gammaRad = computeAngleFromOppositeAndHypotenuse(length, r1);
    float gamma = gammaRad * 180.f / 3.14;

    float alphaRad = computeAngleFromOppositeAndHypotenuse(y, r1);
    float alpha = alphaRad * 180.f / 3.14;

    float thetaRad = alphaRad - gammaRad;
    float theta = thetaRad * 180.f / 3.14;

    if (x < 0)
        theta = 180.f - theta;
    else if (x > 0 && y < 0)
        theta = 360.f + theta;

    std::vector<float> values = { theta, phi1 };
    return values;
}

float RevoluteRevolute::computeAngleFromLength(float a, float b, float c) 
{
    // Uses the cosine rule to compute the angle opposite to C
    float phi = std::acos(
        ( std::pow(a, 2) + std::pow(b, 2) - std::pow(c, 2) ) / (2 * a * b)
    );

    return phi;
}

float RevoluteRevolute::computeLengthFromAngleAndHypotenuse(float hypotenuse, float phi) 
{
    // Uses the sine rule to compute the length opposite to angle phi
    float length = hypotenuse * std::sin(phi);

    return length;
}

float RevoluteRevolute::computeAngleFromOppositeAndHypotenuse(float opposite, float hypotenuse) 
{
    // Implements "SOH", phi = asin(opposite / hypothenuse)
    float phi = std::asin(opposite / hypotenuse);

    return phi;
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
