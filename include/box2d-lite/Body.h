#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

enum class CombineMode
{
    Average,
    Minimum,
    Multiply,
    Maximum
};

struct Material
{
    Material()
    : staticFriction(0.5f)
    , dynamicFriction(0.5f)
    , restitution(0.0f)
    , frictionCombineMode(CombineMode::Average)
    , restitutionCombineMode(CombineMode::Average)
    {
    }

    float staticFriction;
    float dynamicFriction;
    float restitution;
    CombineMode frictionCombineMode;
    CombineMode restitutionCombineMode;
};

struct Body
{
    Body();
    void Set(const glm::vec3& s, float m);

    void AddForce(const glm::vec3& f)
    {
        force += f;
    }

    glm::vec3 position;
    glm::quat rotation;

    glm::vec3 velocity;
    glm::vec3 angularVelocity;

    glm::vec3 force;
    glm::vec3 torque;

    float mass;
    float invMass;

    glm::vec3 size;
    Material material;
    glm::mat3 invI;
};