#include "armComponent.h"

ArmComponent::ArmComponent(
    Vector3d translate, Vector3d rotate, std::vector<Vector3d> points,
    ArmComponentType type, std::shared_ptr<ArmComponent> parent
) : m_translation(translate), m_rotation(rotate), m_points(points)
{
    m_parent = parent;
    setType(type);
    setLocalTransform();
    setGlobalTransform();
}

void ArmComponent::setLocalTransform() 
{
    Matrix r = Matrix::rotate(m_rotation.x, m_rotation.y, m_rotation.z);
    Matrix t = Matrix::translate(m_translation.x, m_translation.y, m_translation.z);

    m_localTransform = r * t;
}

void ArmComponent::setGlobalTransform() 
{
    Matrix transform = m_localTransform;
    std::shared_ptr<ArmComponent> current = m_parent;
    while (current)
    {
        transform = current->getLocalTransform() * transform;
        current = current->getParent();
    }

    m_globalTransform = transform;
}

std::vector<Vector3d> ArmComponent::getTransformedPoints() 
{
    std::vector<Vector3d> updated = {};
    Matrix transform;

    if (m_parent)
        transform = m_parent->getGlobalTransform();
    else
        transform = m_localTransform;

    for (auto p : m_points)
        updated.push_back((transform * p));

    return updated;
}
