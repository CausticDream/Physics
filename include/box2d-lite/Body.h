#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

enum class GeometryType
{
    Box,
    Sphere,
    Capsule
};

struct Geometry
{
    virtual glm::vec3 ComputeI(float m) const = 0;
    virtual GeometryType GetType() const = 0;
};

struct GeometryBox : Geometry
{
    GeometryBox();
    void Set(const glm::vec3& hs, float m);
    glm::vec3 ComputeI(float m) const override;

    GeometryType GetType() const override
    {
        return GeometryType::Box;
    }

    glm::vec3 halfSize;
};

struct GeometrySphere : Geometry
{
    GeometrySphere();
    void Set(float r, float m);
    glm::vec3 ComputeI(float m) const override;

    GeometryType GetType() const override
    {
        return GeometryType::Sphere;
    }

    float radius;
};

struct GeometryCapsule : Geometry
{
    GeometryCapsule();
    void Set(float r, float hh, float m);
    glm::vec3 ComputeI(float m) const override;

    GeometryType GetType() const override
    {
        return GeometryType::Capsule;
    }

    float radius;
    float halfHeight;
};

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

struct Shape
{
    GeometryBox geometry;
    Material material;
};

struct Body
{
    Body();
    void Set(const glm::vec3& s, float m);
    void AddForce(const glm::vec3& f);

    glm::vec3 position;
    glm::quat rotation;
    glm::vec3 velocity;
    glm::vec3 angularVelocity;
    glm::vec3 force;
    glm::vec3 torque;
    float mass;
    float invMass;
    Shape shape;
    glm::mat3 invI;
};