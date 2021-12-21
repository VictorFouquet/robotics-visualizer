#pragma once

#include <vector>

#include "vector3d.h"
#include "matrix.h"


enum ArmComponentType 
{
    revolute  = 1,
    prismatic = 2,
    joint     = 3,
    ground    = 8,
    link      = 32
};

class ArmComponent
{
public:
    ArmComponent() = default;
    ArmComponent(Vector3d translate, Vector3d rotate);

    ~ArmComponent() = default;

    inline void setType(ArmComponentType type) { m_type |= type; }
    inline bool isType(ArmComponentType type) { return (m_type & type); }

    inline void setChild(ArmComponent* child) { m_child = child; }
    inline void setParent(ArmComponent* parent) { m_parent = parent; }
    inline ArmComponent* getChild() { return m_child; }
    inline ArmComponent* getParent() { return m_parent; }

    inline void setTranslation(Vector3d translate) { m_translation = translate; }
    inline void setRotation(Vector3d rotate) { m_rotation = rotate; }

    void setLocalTransform();
    void setGlobalTransform();
    inline Matrix getLocalTransform() { return m_localTransform; }
    inline Matrix getGlobalTransform() { return m_globalTransform; }

    inline void setPoints(std::vector<Vector3d> points) { m_points = points; }
    inline std::vector<Vector3d> getPoints() { return m_points; }
    std::vector<Vector3d> getTransformedPoints();

private:
    int m_type = 0;
    ArmComponent* m_parent;
    ArmComponent* m_child;

    std::vector<Vector3d> m_points;
    Vector3d m_translation;
    Vector3d m_rotation;
    Matrix m_localTransform;
    Matrix m_globalTransform;
};
