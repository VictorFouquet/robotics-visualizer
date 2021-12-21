#include "armComponent.h"

void ArmComponent::setLocalTransform() 
{
    Matrix r = Matrix::rotate(m_rotation.x, m_rotation.y, m_rotation.z);
    Matrix t = Matrix::translate(m_translation.x, m_translation.y, m_translation.z);

    m_localTransform = r * t;
}

void ArmComponent::setGlobalTransform() 
{
    Matrix transform = m_localTransform;
    ArmComponent* current = m_parent;
    while (current)
    {
        transform = current->getLocalTransform() * transform;
        current = current->getParent();
    }

    m_globalTransform = transform;
}

std::vector<Vector3d> ArmComponent::getTransformedPoints() 
{
    Matrix transform = m_parent->getGlobalTransform();
    std::vector<Vector3d> updated = {};

    for (auto p : m_points)
        updated.push_back((transform * p));
    
    return updated;
}
