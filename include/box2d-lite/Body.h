#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

struct Geometry
{
    Geometry();
    glm::mat3 invI;
};

struct GeometryBox : Geometry
{
    GeometryBox();
    void Set(const glm::vec3& hs, float m);

    glm::vec3 halfSize;
};

struct GeometrySphere : Geometry
{
    GeometrySphere();
    void Set(float r, float m);

    float radius;
};

struct GeometryCapsule : Geometry
{
    GeometryCapsule();
    void Set(float r, float hh, float m);

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
};