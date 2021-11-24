#include "robotArmExample.h"

RevoluteRevolute::RevoluteRevolute(float lenghtLink1, float lenghtLink2, float theta, float phi)
    : m_lenghtLink1(lenghtLink1), m_lenghtLink2(lenghtLink2), m_theta(theta), m_phi(phi)
{
    std::vector<float> links = { phi, theta };

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
