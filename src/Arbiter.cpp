#include "Arbiter.h"
#include "Body.h"
#include "World.h"

Arbiter::Arbiter(Shape* shape1, Shape* shape2)
{
    Shape* firstShape;
    Shape* secondShape;
    if (shape1->GetUniqueID() < shape2->GetUniqueID())
    {
        firstShape = shape1;
        secondShape = shape2;
    }
    else
    {
        firstShape = shape2;
        secondShape = shape1;
    }
    m_body1 = firstShape->m_owner;
    m_body2 = secondShape->m_owner;

    m_contactCount = Collide(m_contacts, m_body1, firstShape, m_body2, secondShape);

    const CombineMode effectiveFrictionCombineMode = std::max(firstShape->m_material->m_frictionCombineMode, secondShape->m_material->m_frictionCombineMode);
    switch (effectiveFrictionCombineMode)
    {
        case CombineMode::Average:
        {
            m_staticFriction = (firstShape->m_material->m_staticFriction + secondShape->m_material->m_staticFriction) * 0.5f;
            m_dynamicFriction = (firstShape->m_material->m_dynamicFriction + secondShape->m_material->m_dynamicFriction) * 0.5f;
            break;
        }

        case CombineMode::Minimum:
        {
            m_staticFriction = std::min(firstShape->m_material->m_staticFriction, secondShape->m_material->m_staticFriction);
            m_dynamicFriction = std::min(firstShape->m_material->m_dynamicFriction, secondShape->m_material->m_dynamicFriction);
            break;
        }

        case CombineMode::Multiply:
        {
            m_staticFriction = firstShape->m_material->m_staticFriction * secondShape->m_material->m_staticFriction;
            m_dynamicFriction = firstShape->m_material->m_dynamicFriction * secondShape->m_material->m_dynamicFriction;
            break;
        }

        case CombineMode::Maximum:
        {
            m_staticFriction = std::max(firstShape->m_material->m_staticFriction, secondShape->m_material->m_staticFriction);
            m_dynamicFriction = std::max(firstShape->m_material->m_dynamicFriction, secondShape->m_material->m_dynamicFriction);
            break;
        }

        default:
        {
            assert(false);
        }
    }

    const CombineMode effectiveRestitutionCombineMode = std::max(firstShape->m_material->m_restitutionCombineMode, secondShape->m_material->m_restitutionCombineMode);
    switch (effectiveRestitutionCombineMode)
    {
        case CombineMode::Average:
        {
            m_restitution = (firstShape->m_material->m_restitution + secondShape->m_material->m_restitution) * 0.5f;
            break;
        }

        case CombineMode::Minimum:
        {
            m_restitution = std::min(firstShape->m_material->m_restitution, secondShape->m_material->m_restitution);
            break;
        }

        case CombineMode::Multiply:
        {
            m_restitution = firstShape->m_material->m_restitution * secondShape->m_material->m_restitution;
            break;
        }

        case CombineMode::Maximum:
        {
            m_restitution = std::max(firstShape->m_material->m_restitution, secondShape->m_material->m_restitution);
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
            c->m_Pnb = cOld->m_Pnb;
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
        if ((1.0f - std::abs(glm::dot(c->m_normal, glm::vec3(1.0f, 0.0f, 0.0f)))) < 1.0e-4f)
        {
            tangent = glm::normalize(glm::cross(c->m_normal, glm::vec3(0.0f, 1.0f, 0.0f)));
        }
        else
        {
            tangent = glm::normalize(glm::cross(c->m_normal, glm::vec3(1.0f, 0.0f, 0.0f)));
        }
        glm::vec3 rt1 = glm::cross(r1, tangent);
        glm::vec3 rt2 = glm::cross(r2, tangent);
        float kTangent = m_body1->m_invMass + m_body2->m_invMass;
        kTangent += glm::dot(rt1, m_body1->m_invI * rt1);
        kTangent += glm::dot(rt2, m_body2->m_invI * rt2);
        c->m_massTangent = 1.0f / kTangent;

        constexpr float k_biasFactor = 0.1f;
        constexpr float k_allowedPenetration = 0.01f;
        c->m_bias = -k_biasFactor * invElapsedTime * glm::min(0.0f, c->m_separation + k_allowedPenetration);

        constexpr float velocityThreshold = 1.0f;
        glm::vec3 dv = m_body2->m_velocity + glm::cross(m_body2->m_angularVelocity, r2) - m_body1->m_velocity - glm::cross(m_body1->m_angularVelocity, r1);
        float vn = glm::dot(dv, c->m_normal);
        if (vn < -velocityThreshold)
        {
            c->m_bias -= m_restitution * vn;
        }

        // Apply normal + friction impulse.
        glm::vec3 P = c->m_Pn * c->m_normal + c->m_Pt * tangent;

        m_body1->m_velocity -= m_body1->m_invMass * P;
        m_body1->m_angularVelocity -= m_body1->m_invI * glm::cross(r1, P);

        m_body2->m_velocity += m_body2->m_invMass * P;
        m_body2->m_angularVelocity += m_body2->m_invI * glm::cross(r2, P);
    }
}

void Arbiter::ApplyImpulse()
{
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

        // Relative velocity at contact.
        dv = m_body2->m_velocity + glm::cross(m_body2->m_angularVelocity, c->m_r2) - m_body1->m_velocity - glm::cross(m_body1->m_angularVelocity, c->m_r1);

        glm::vec3 tangent;
        if ((1.0f - std::abs(glm::dot(c->m_normal, glm::vec3(1.0f, 0.0f, 0.0f)))) < 1.0e-4f)
        {
            tangent = glm::normalize(glm::cross(c->m_normal, glm::vec3(0.0f, 1.0f, 0.0f)));
        }
        else
        {
            tangent = glm::normalize(glm::cross(c->m_normal, glm::vec3(1.0f, 0.0f, 0.0f)));
        }
        float vt = glm::dot(dv, tangent);
        float dPt = c->m_massTangent * (-vt);

        constexpr float velocityThreshold = 1.0f;
        const float effectiveFriction = (std::abs(vt) < velocityThreshold) ? m_staticFriction : m_dynamicFriction;

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
    }
}