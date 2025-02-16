#pragma once

#include <glm/glm.hpp>

struct Body;

struct Joint
{
    Joint();
    void Set(Body* body1, Body* body2, const glm::vec3& anchor);
    void PreStep(float invElapsedTime);
    void ApplyImpulse();

    glm::mat3 m_M;
    glm::vec3 m_localAnchor1;
    glm::vec3 m_localAnchor2;
    glm::vec3 m_r1;
    glm::vec3 m_r2;
    glm::vec3 m_bias;
    glm::vec3 m_P;
    Body* m_body1;
    Body* m_body2;
    float m_biasFactor;
    float m_softness;
};