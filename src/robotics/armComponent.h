#pragma once

#include <vector>
#include <memory>

#include "vector3d.h"
#include "matrix.h"


enum ArmComponentType 
{
    revolute    = 1,
    prismatic   = 2,
    fixed       = 4,
    joint       = 7,
    ground      = 8,
    rigidBody   = 16,
    endEffector = 32,
    link        = 56
};

class ArmComponent
{
public:
    ArmComponent() = default;
    ArmComponent(Vector3d translate, Vector3d rotate)
        : m_translation(translate), m_rotation(rotate) { setLocalTransform(); }

    ArmComponent(Vector3d translate, Vector3d rotate, std::vector<Vector3d> points, ArmComponentType type);

    ~ArmComponent() = default;

    inline void setType(ArmComponentType type) { m_type |= type; }
    inline bool isType(ArmComponentType type) { return (m_type & type); }
    inline int getType() { return m_type; }
    
    inline void setChild(std::shared_ptr<ArmComponent> child) { m_child = child; }
    inline void setParent(std::shared_ptr<ArmComponent> parent) { m_parent = parent; }
    inline std::shared_ptr<ArmComponent> getChild() { return m_child; }
    inline std::shared_ptr<ArmComponent> getParent() { return m_parent; }

    inline void setTranslation(Vector3d translate) { m_translation = translate; }
    inline void setRotation(Vector3d rotate) { m_rotation = rotate; }
    inline Vector3d getRotation() { return m_rotation; }
    inline Vector3d getTranslation() { return m_translation; }

    void setLocalTransform();
    void setGlobalTransform();
    inline Matrix getLocalTransform() { return m_localTransform; }
    inline Matrix getGlobalTransform() { return m_globalTransform; }

    inline void setPoints(std::vector<Vector3d> points) { m_points = points; }
    inline std::vector<Vector3d> getPoints() { return m_points; }
    std::vector<Vector3d> getTransformedPoints();

private:
    int m_type = 0;
    std::shared_ptr<ArmComponent> m_parent = nullptr;
    std::shared_ptr<ArmComponent> m_child = nullptr;

    std::vector<Vector3d> m_points;
    Vector3d m_translation;
    Vector3d m_rotation;
    Matrix m_localTransform;
    Matrix m_globalTransform;
};
