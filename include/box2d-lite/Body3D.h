#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

struct Body3D
{
    Body3D();
    void Set(const glm::vec3& w, float m);

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

    glm::vec3 width;

    float friction;
    float mass, invMass;
    glm::mat3 I, invI;
};