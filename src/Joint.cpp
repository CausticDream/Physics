#include "Joint.h"
#include "Body.h"
#include "World.h"

Joint::Joint(JointType type)
: m_type(type)
{
}

JointSpherical::JointSpherical()
: Joint(JointType::Spherical)
, m_body1(nullptr)
, m_body2(nullptr)
, m_P(0.0f, 0.0f, 0.0f)
, m_biasFactor(0.2f)
, m_softness(0.0f)
{
}

void JointSpherical::Set(Body* b1, Body* b2, const glm::vec3& anchor)
{
    m_body1 = b1;
    m_body2 = b2;

    m_localAnchor1 = glm::conjugate(m_body1->m_rotation) * (anchor - m_body1->m_position);
    m_localAnchor2 = glm::conjugate(m_body2->m_rotation) * (anchor - m_body2->m_position);

    m_P = glm::vec3(0.0f, 0.0f, 0.0f);

    m_softness = 0.0f;
    m_biasFactor = 0.1f;
}

void JointSpherical::PreStep(float invElapsedTime)
{
    m_r1 = m_body1->m_rotation * m_localAnchor1;
    m_r2 = m_body2->m_rotation * m_localAnchor2;

    glm::mat3 K1 = glm::mat3(m_body1->m_invMass + m_body2->m_invMass);

    glm::mat3 skewR1 = glm::mat3(
        0.0f, -m_r1.z, m_r1.y,
        m_r1.z, 0.0f, -m_r1.x,
        -m_r1.y, m_r1.x, 0.0f
    );
    glm::mat3 K2 = skewR1 * m_body1->m_invI * glm::transpose(skewR1);

    glm::mat3 skewR2 = glm::mat3(
        0.0f, -m_r2.z, m_r2.y,
        m_r2.z, 0.0f, -m_r2.x,
        -m_r2.y, m_r2.x, 0.0f
    );
    glm::mat3 K3 = skewR2 * m_body2->m_invI * glm::transpose(skewR2);

    glm::mat3 K = K1 + K2 + K3;
    K[0].x += m_softness;
    K[1].y += m_softness;
    K[2].z += m_softness;

    m_M = glm::inverse(K);

    glm::vec3 p1 = m_body1->m_position + m_r1;
    glm::vec3 p2 = m_body2->m_position + m_r2;
    glm::vec3 dp = p2 - p1;

    m_bias = -m_biasFactor * invElapsedTime * dp;

    m_body1->m_velocity -= m_body1->m_invMass * m_P;
    m_body1->m_angularVelocity -= m_body1->m_invI * glm::cross(m_r1, m_P);

    m_body2->m_velocity += m_body2->m_invMass * m_P;
    m_body2->m_angularVelocity += m_body2->m_invI * glm::cross(m_r2, m_P);
}

void JointSpherical::ApplyImpulse()
{
    glm::vec3 dv = m_body2->m_velocity + glm::cross(m_body2->m_angularVelocity, m_r2) - m_body1->m_velocity - glm::cross(m_body1->m_angularVelocity, m_r1);
    glm::vec3 impulse = m_M * (m_bias - dv - m_softness * m_P);

    m_body1->m_velocity -= m_body1->m_invMass * impulse;
    m_body1->m_angularVelocity -= m_body1->m_invI * glm::cross(m_r1, impulse);

    m_body2->m_velocity += m_body2->m_invMass * impulse;
    m_body2->m_angularVelocity += m_body2->m_invI * glm::cross(m_r2, impulse);

    m_P += impulse;
}

JointHinge::JointHinge()
: Joint(JointType::Hinge)
, m_body1(nullptr)
, m_body2(nullptr)
, m_P(0.0f, 0.0f, 0.0f)
, m_angularImpulse(0.0f)
, m_biasFactor(0.2f)
, m_softness(0.0f)
{
}

void JointHinge::Set(Body* b1, Body* b2, const glm::vec3& anchor, const glm::vec3& axis)
{
    m_body1 = b1;
    m_body2 = b2;

    m_localAnchor1 = glm::conjugate(m_body1->m_rotation) * (anchor - m_body1->m_position);
    m_localAnchor2 = glm::conjugate(m_body2->m_rotation) * (anchor - m_body2->m_position);

    m_localAxis1 = glm::conjugate(m_body1->m_rotation) * axis;
    m_localAxis2 = glm::conjugate(m_body2->m_rotation) * axis;

    m_P = glm::vec3(0.0f, 0.0f, 0.0f);
    m_angularImpulse = 0.0f;

    m_softness = 0.0f;
    m_biasFactor = 0.1f;
}

void JointHinge::PreStep(float invElapsedTime)
{
    m_r1 = m_body1->m_rotation * m_localAnchor1;
    m_r2 = m_body2->m_rotation * m_localAnchor2;
    m_a1 = m_body1->m_rotation * m_localAxis1;
    m_a2 = m_body2->m_rotation * m_localAxis2;

    glm::mat3 K1 = glm::mat3(m_body1->m_invMass + m_body2->m_invMass);

    glm::mat3 skewR1 = glm::mat3(
        0.0f, -m_r1.z, m_r1.y,
        m_r1.z, 0.0f, -m_r1.x,
        -m_r1.y, m_r1.x, 0.0f
    );
    glm::mat3 K2 = skewR1 * m_body1->m_invI * glm::transpose(skewR1);

    glm::mat3 skewR2 = glm::mat3(
        0.0f, -m_r2.z, m_r2.y,
        m_r2.z, 0.0f, -m_r2.x,
        -m_r2.y, m_r2.x, 0.0f
    );
    glm::mat3 K3 = skewR2 * m_body2->m_invI * glm::transpose(skewR2);

    glm::mat3 K = K1 + K2 + K3;
    K[0].x += m_softness;
    K[1].y += m_softness;
    K[2].z += m_softness;

    m_M = glm::inverse(K);

    glm::vec3 p1 = m_body1->m_position + m_r1;
    glm::vec3 p2 = m_body2->m_position + m_r2;
    glm::vec3 dp = p2 - p1;

    m_bias = -m_biasFactor * invElapsedTime * dp;

    m_angularMass = 1.0f / (glm::dot(m_a1, m_body1->m_invI * m_a1) + glm::dot(m_a2, m_body2->m_invI * m_a2));

    float angle = glm::acos(glm::dot(m_a1, m_a2));
    m_angularBias = -m_biasFactor * invElapsedTime * angle;

    m_body1->m_velocity -= m_body1->m_invMass * m_P;
    m_body1->m_angularVelocity -= m_body1->m_invI * (glm::cross(m_r1, m_P) + m_angularImpulse * m_a1);

    m_body2->m_velocity += m_body2->m_invMass * m_P;
    m_body2->m_angularVelocity += m_body2->m_invI * (glm::cross(m_r2, m_P) + m_angularImpulse * m_a2);
}

void JointHinge::ApplyImpulse()
{
    glm::vec3 dv = m_body2->m_velocity + glm::cross(m_body2->m_angularVelocity, m_r2) - m_body1->m_velocity - glm::cross(m_body1->m_angularVelocity, m_r1);
    glm::vec3 impulse = m_M * (m_bias - dv - m_softness * m_P);

    m_body1->m_velocity -= m_body1->m_invMass * impulse;
    m_body1->m_angularVelocity -= m_body1->m_invI * glm::cross(m_r1, impulse);

    m_body2->m_velocity += m_body2->m_invMass * impulse;
    m_body2->m_angularVelocity += m_body2->m_invI * glm::cross(m_r2, impulse);

    m_P += impulse;

    float Cdot = glm::dot(m_a2, m_body2->m_angularVelocity) - glm::dot(m_a1, m_body1->m_angularVelocity);
    float impulseAngular = m_angularMass * (-Cdot + m_angularBias - m_softness * m_angularImpulse);

    m_body1->m_angularVelocity -= m_body1->m_invI * impulseAngular * m_a1;
    m_body2->m_angularVelocity += m_body2->m_invI * impulseAngular * m_a2;

    m_angularImpulse += impulseAngular;
}