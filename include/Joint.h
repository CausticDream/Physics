#pragma once

#include <glm/glm.hpp>

struct Body;

enum class JointType
{
    Spherical,
    Hinge
};

struct Joint
{
    JointType GetType() const
    {
        return m_type;
    }

protected:
    Joint(JointType type);
    JointType m_type;
};

struct JointSpherical : Joint
{
    JointSpherical();
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

struct JointHinge : Joint
{
    JointHinge();
    void Set(Body* body1, Body* body2, const glm::vec3& anchor, const glm::vec3& axis);
    void PreStep(float invElapsedTime);
    void ApplyImpulse();

    glm::mat3 m_M;
    float m_angularMass;
    glm::vec3 m_localAnchor1;
    glm::vec3 m_localAnchor2;
    glm::vec3 m_localAxis1;
    glm::vec3 m_localAxis2;
    glm::vec3 m_r1;
    glm::vec3 m_r2;
    glm::vec3 m_a1;
    glm::vec3 m_a2;
    glm::vec3 m_bias;
    float m_angularBias;
    glm::vec3 m_P;
    float m_angularImpulse;
    Body* m_body1;
    Body* m_body2;
    float m_biasFactor;
    float m_softness;
};