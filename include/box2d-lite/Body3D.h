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

struct Body3D
{
    Body3D();
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

    glm::vec3 size;

    float staticFriction;
    float dynamicFriction;
    float restitution;
    CombineMode frictionCombineMode;
    CombineMode restitutionCombineMode;

    float mass, invMass;
    glm::mat3 invI;
};