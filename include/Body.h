#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>

struct Body;

enum class CombineMode
{
    Average,
    Minimum,
    Multiply,
    Maximum
};

struct Material
{
    Material();

    float m_staticFriction;
    float m_dynamicFriction;
    float m_restitution;
    CombineMode m_frictionCombineMode;
    CombineMode m_restitutionCombineMode;
};

enum class ShapeType : size_t
{
    Box,
    Sphere,
    Capsule,
    Count
};

struct Shape
{
    ~Shape();

    Body* m_owner;
    glm::vec3 m_position;
    glm::quat m_rotation;
    Material* m_material;

    uint32_t GetUniqueID() const
    {
        return m_uniqueID;
    }

    ShapeType GetType() const
    {
        return m_type;
    }

protected:
    Shape(ShapeType type);

    uint32_t m_uniqueID;
    ShapeType m_type;
};

struct ShapeBox : Shape
{
    ShapeBox();
    void Set(const glm::vec3& halfSize);
    glm::vec3 ComputeI(float mass) const;

    glm::vec3 m_halfSize;
};

struct ShapeSphere : Shape
{
    ShapeSphere();
    void Set(float radius);
    glm::vec3 ComputeI(float mass) const;

    float m_radius;
};

struct ShapeCapsule : Shape
{
    ShapeCapsule();
    void Set(float radius, float halfHeight);
    glm::vec3 ComputeI(float mass) const;

    float m_radius;
    float m_halfHeight;
};

struct Body
{
    Body();
    ~Body();
    void SetMass(float mass);
    void AddForce(const glm::vec3& force);
    void AddShape(Shape* shape);
    void ComputeInvI();

    glm::vec3 m_position;
    glm::quat m_rotation;
    glm::vec3 m_velocity;
    glm::vec3 m_angularVelocity;
    glm::vec3 m_force;
    glm::vec3 m_torque;
    float m_mass;
    float m_invMass;
    std::vector<Shape*> m_shapes;
    glm::mat3 m_invI;
};