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

    float staticFriction;
    float dynamicFriction;
    float restitution;
    CombineMode frictionCombineMode;
    CombineMode restitutionCombineMode;
};

enum class ShapeType
{
    Box,
    Sphere,
    Capsule
};

struct Shape
{
    ~Shape();

    Body* owner;
    Material* material;

    uint32_t GetUniqueID() const
    {
        return uniqueID;
    }

    ShapeType GetType() const
    {
        return type;
    }

protected:
    Shape(ShapeType type);

    uint32_t uniqueID;
    ShapeType type;
};

struct ShapeBox : Shape
{
    ShapeBox();
    void Set(const glm::vec3& hs);

    glm::vec3 halfSize;
};

struct ShapeSphere : Shape
{
    ShapeSphere();
    void Set(float r);

    float radius;
};

struct ShapeCapsule : Shape
{
    ShapeCapsule();
    void Set(float r, float hh);

    float radius;
    float halfHeight;
};

struct Body
{
    Body();
    ~Body();
    void SetMass(float m);
    void AddForce(const glm::vec3& f);
    void AddShape(Shape* shape);
    void UpdateInvI();

    glm::vec3 position;
    glm::quat rotation;
    glm::vec3 velocity;
    glm::vec3 angularVelocity;
    glm::vec3 force;
    glm::vec3 torque;
    float mass;
    float invMass;
    std::vector<Shape*> shapes;
    glm::mat3 invI;
};