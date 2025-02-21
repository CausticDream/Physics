#include "Arbiter.h"
#include "Body.h"
#include "World.h"

constexpr float velocityThreshold = 1.0f;

void ComputeBasis(const glm::vec3& a, glm::vec3& b, glm::vec3& c)
{
    // Suppose vector a has all equal components and is a unit vector:
    // a = (s, s, s)
    // Then 3*s*s = 1, s = sqrt(1/3) = 0.57735. This means that at
    // least one component of a unit vector must be greater or equal
    // to 0.57735.

    if (std::abs(a.x) >= 0.57735f)
        b = glm::vec3(a.y, -a.x, 0.0f);
    else
        b = glm::vec3(0.0f, a.z, -a.y);

    b = glm::normalize(b);
    c = glm::cross(a, b);
}

Arbiter::Arbiter(Shape* shape1, Shape* shape2)
{
    Shape* lowestShape;
    Shape* highestShape;
    if (shape1->GetUniqueID() < shape2->GetUniqueID())
    {
        lowestShape = shape1;
        highestShape = shape2;
    }
    else
    {
        lowestShape = shape2;
        highestShape = shape1;
    }
    m_body1 = lowestShape->m_owner;
    m_body2 = highestShape->m_owner;
    m_isTrigger = shape1->IsTrigger() || shape2->IsTrigger();

    m_contactCount = Collide(m_contacts, m_body1, lowestShape, m_body2, highestShape);

    const CombineMode effectiveFrictionCombineMode = std::max(lowestShape->m_material->m_frictionCombineMode, highestShape->m_material->m_frictionCombineMode);
    switch (effectiveFrictionCombineMode)
    {
        case CombineMode::Average:
        {
            m_staticFriction = (lowestShape->m_material->m_staticFriction + highestShape->m_material->m_staticFriction) * 0.5f;
            m_dynamicFriction = (lowestShape->m_material->m_dynamicFriction + highestShape->m_material->m_dynamicFriction) * 0.5f;
            break;
        }

        case CombineMode::Minimum:
        {
            m_staticFriction = std::min(lowestShape->m_material->m_staticFriction, highestShape->m_material->m_staticFriction);
            m_dynamicFriction = std::min(lowestShape->m_material->m_dynamicFriction, highestShape->m_material->m_dynamicFriction);
            break;
        }

        case CombineMode::Multiply:
        {
            m_staticFriction = lowestShape->m_material->m_staticFriction * highestShape->m_material->m_staticFriction;
            m_dynamicFriction = lowestShape->m_material->m_dynamicFriction * highestShape->m_material->m_dynamicFriction;
            break;
        }

        case CombineMode::Maximum:
        {
            m_staticFriction = std::max(lowestShape->m_material->m_staticFriction, highestShape->m_material->m_staticFriction);
            m_dynamicFriction = std::max(lowestShape->m_material->m_dynamicFriction, highestShape->m_material->m_dynamicFriction);
            break;
        }

        default:
        {
            assert(false);
        }
    }

    const CombineMode effectiveRestitutionCombineMode = std::max(lowestShape->m_material->m_restitutionCombineMode, highestShape->m_material->m_restitutionCombineMode);
    switch (effectiveRestitutionCombineMode)
    {
        case CombineMode::Average:
        {
            m_restitution = (lowestShape->m_material->m_restitution + highestShape->m_material->m_restitution) * 0.5f;
            break;
        }

        case CombineMode::Minimum:
        {
            m_restitution = std::min(lowestShape->m_material->m_restitution, highestShape->m_material->m_restitution);
            break;
        }

        case CombineMode::Multiply:
        {
            m_restitution = lowestShape->m_material->m_restitution * highestShape->m_material->m_restitution;
            break;
        }

        case CombineMode::Maximum:
        {
            m_restitution = std::max(lowestShape->m_material->m_restitution, highestShape->m_material->m_restitution);
            break;
        }

        default:
        {
            assert(false);
        }
    }
}

void Arbiter::Update(Contact* contacts, size_t contactCount, Contact* newContacts, size_t& newContactCount)
{
    newContactCount = 0;
    Contact mergedContacts[g_maxContactPoints];

    for (size_t i = 0; i < contactCount; ++i)
    {
        Contact* cNew = contacts + i;

        size_t k = std::numeric_limits<size_t>::max();
        for (size_t j = 0; j < m_contactCount; ++j)
        {
            Contact* cOld = m_contacts + j;
            if (cNew->m_feature == cOld->m_feature)
            {
                k = j;
                break;
            }
        }

        if (k != std::numeric_limits<size_t>::max())
        {
            Contact* c = mergedContacts + i;
            Contact* cOld = m_contacts + k;
            *c = *cNew;
            c->m_Pn = cOld->m_Pn;
            c->m_Pt = cOld->m_Pt;
            c->m_Pb = cOld->m_Pb;
        }
        else
        {
            mergedContacts[i] = contacts[i];

            newContacts[newContactCount] = contacts[i];
            ++newContactCount;
        }
    }

    for (size_t i = 0; i < contactCount; ++i)
    {
        m_contacts[i] = mergedContacts[i];
    }

    m_contactCount = contactCount;
}

void Arbiter::PreStep(float invElapsedTime)
{
    if (m_isTrigger)
    {
        return;
    }

    for (size_t i = 0; i < m_contactCount; ++i)
    {
        Contact* c = m_contacts + i;

        glm::vec3 r1 = c->m_position - m_body1->m_position;
        glm::vec3 r2 = c->m_position - m_body2->m_position;

        // Precompute normal mass, tangent mass, and bias.
        glm::vec3 rn1 = glm::cross(r1, c->m_normal);
        glm::vec3 rn2 = glm::cross(r2, c->m_normal);
        float kNormal = m_body1->m_invMass + m_body2->m_invMass;
        kNormal += glm::dot(rn1, m_body1->m_invI * rn1);
        kNormal += glm::dot(rn2, m_body2->m_invI * rn2);
        c->m_massNormal = 1.0f / kNormal;

        glm::vec3 tangent;
        glm::vec3 bitangent;
        ComputeBasis(c->m_normal, tangent, bitangent);

        glm::vec3 rt1 = glm::cross(r1, tangent);
        glm::vec3 rt2 = glm::cross(r2, tangent);
        float kTangent = m_body1->m_invMass + m_body2->m_invMass;
        kTangent += glm::dot(rt1, m_body1->m_invI * rt1);
        kTangent += glm::dot(rt2, m_body2->m_invI * rt2);
        c->m_massTangent = 1.0f / kTangent;

        glm::vec3 rb1 = glm::cross(r1, bitangent);
        glm::vec3 rb2 = glm::cross(r2, bitangent);
        float kBitangent = m_body1->m_invMass + m_body2->m_invMass;
        kBitangent += glm::dot(rb1, m_body1->m_invI * rb1);
        kBitangent += glm::dot(rb2, m_body2->m_invI * rb2);
        c->m_massBitangent = 1.0f / kBitangent;

        constexpr float k_biasFactor = 0.1f;
        constexpr float k_allowedPenetration = 0.01f;
        c->m_bias = -k_biasFactor * invElapsedTime * glm::min(0.0f, c->m_separation + k_allowedPenetration);

        glm::vec3 dv = m_body2->m_velocity + glm::cross(m_body2->m_angularVelocity, r2) - m_body1->m_velocity - glm::cross(m_body1->m_angularVelocity, r1);
        float vn = glm::dot(dv, c->m_normal);
        if (vn < -velocityThreshold)
        {
            c->m_bias -= m_restitution * vn;
        }

        // Apply normal + friction impulse.
        glm::vec3 P = (c->m_Pn * c->m_normal) + (c->m_Pt * tangent) + (c->m_Pb * bitangent);

        m_body1->m_velocity -= m_body1->m_invMass * P;
        m_body1->m_angularVelocity -= m_body1->m_invI * glm::cross(r1, P);

        m_body2->m_velocity += m_body2->m_invMass * P;
        m_body2->m_angularVelocity += m_body2->m_invI * glm::cross(r2, P);
    }
}

void Arbiter::ApplyImpulse()
{
    if (m_isTrigger)
    {
        return;
    }

    for (size_t i = 0; i < m_contactCount; ++i)
    {
        Contact* c = m_contacts + i;

        c->m_r1 = c->m_position - m_body1->m_position;
        c->m_r2 = c->m_position - m_body2->m_position;

        // Relative velocity at contact.
        glm::vec3 dv = m_body2->m_velocity + glm::cross(m_body2->m_angularVelocity, c->m_r2) - m_body1->m_velocity - glm::cross(m_body1->m_angularVelocity, c->m_r1);

        // Compute normal impulse.
        float vn = glm::dot(dv, c->m_normal);
        float dPn = c->m_massNormal * (-vn + c->m_bias);

        // Clamp the accumulated impulse.
        float Pn0 = c->m_Pn;
        c->m_Pn = glm::max(Pn0 + dPn, 0.0f);
        dPn = c->m_Pn - Pn0;

        // Apply contact impulse.
        glm::vec3 Pn = dPn * c->m_normal;

        m_body1->m_velocity -= m_body1->m_invMass * Pn;
        m_body1->m_angularVelocity -= m_body1->m_invI * glm::cross(c->m_r1, Pn);

        m_body2->m_velocity += m_body2->m_invMass * Pn;
        m_body2->m_angularVelocity += m_body2->m_invI * glm::cross(c->m_r2, Pn);

        glm::vec3 tangent;
        glm::vec3 bitangent;
        ComputeBasis(c->m_normal, tangent, bitangent);

        // Relative velocity at contact.
        dv = m_body2->m_velocity + glm::cross(m_body2->m_angularVelocity, c->m_r2) - m_body1->m_velocity - glm::cross(m_body1->m_angularVelocity, c->m_r1);

        float vt = glm::dot(dv, tangent);
        float dPt = c->m_massTangent * (-vt);

        float effectiveFriction = (std::abs(vt) < velocityThreshold) ? m_staticFriction : m_dynamicFriction;

        // Compute friction impulse.
        float maxPt = effectiveFriction * c->m_Pn;

        // Clamp friction.
        float oldTangentImpulse = c->m_Pt;
        c->m_Pt = glm::clamp(oldTangentImpulse + dPt, -maxPt, maxPt);
        dPt = c->m_Pt - oldTangentImpulse;

        // Apply contact impulse.
        glm::vec3 Pt = dPt * tangent;

        m_body1->m_velocity -= m_body1->m_invMass * Pt;
        m_body1->m_angularVelocity -= m_body1->m_invI * glm::cross(c->m_r1, Pt);

        m_body2->m_velocity += m_body2->m_invMass * Pt;
        m_body2->m_angularVelocity += m_body2->m_invI * glm::cross(c->m_r2, Pt);

        // Relative velocity at contact.
        dv = m_body2->m_velocity + glm::cross(m_body2->m_angularVelocity, c->m_r2) - m_body1->m_velocity - glm::cross(m_body1->m_angularVelocity, c->m_r1);

        float vb = glm::dot(dv, bitangent);
        float dPb = c->m_massBitangent * (-vb);

        effectiveFriction = (std::abs(vb) < velocityThreshold) ? m_staticFriction : m_dynamicFriction;

        // Compute friction impulse.
        float maxPb = effectiveFriction * c->m_Pn;

        // Clamp friction.
        float oldBitangentImpulse = c->m_Pb;
        c->m_Pb = glm::clamp(oldBitangentImpulse + dPb, -maxPb, maxPb);
        dPb = c->m_Pb - oldBitangentImpulse;

        // Apply contact impulse.
        glm::vec3 Pb = dPb * bitangent;

        m_body1->m_velocity -= m_body1->m_invMass * Pb;
        m_body1->m_angularVelocity -= m_body1->m_invI * glm::cross(c->m_r1, Pb);

        m_body2->m_velocity += m_body2->m_invMass * Pb;
        m_body2->m_angularVelocity += m_body2->m_invI * glm::cross(c->m_r2, Pb);
    }
}