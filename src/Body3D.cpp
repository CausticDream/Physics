#include "box2d-lite/Body3D.h"

Body3D::Body3D()
{
    position = glm::vec3(0.0f, 0.0f, 0.0f);
    rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    velocity = glm::vec3(0.0f, 0.0f, 0.0f);
    angularVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
    force = glm::vec3(0.0f, 0.0f, 0.0f);
    torque = glm::vec3(0.0f, 0.0f, 0.0f);
    mass = FLT_MAX;
    invMass = 0.0f;

    size = glm::vec3(1.0f, 1.0f, 1.0f);
    invI = glm::mat3(0.0f);
}

void Body3D::Set(const glm::vec3& s, float m)
{
    position = glm::vec3(0.0f, 0.0f, 0.0f);
    rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    velocity = glm::vec3(0.0f, 0.0f, 0.0f);
    angularVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
    force = glm::vec3(0.0f, 0.0f, 0.0f);
    torque = glm::vec3(0.0f, 0.0f, 0.0f);

    size = s;
    mass = m;

    if (mass < FLT_MAX)
    {
        invMass = 1.0f / mass;
        glm::vec3 I = glm::vec3(mass * (size.y * size.y + size.z * size.z) / 12.0f,
                                mass * (size.x * size.x + size.z * size.z) / 12.0f,
                                mass * (size.x * size.x + size.y * size.y) / 12.0f);
        invI = glm::mat3(glm::vec3(1.0f / I.x, 0.0f, 0.0f),
                         glm::vec3(0.0f, 1.0f / I.y, 0.0f),
                         glm::vec3(0.0f, 0.0f, 1.0f / I.z));
    }
    else
    {
        invMass = 0.0f;
        invI = glm::mat3(0.0f);
    }
}
