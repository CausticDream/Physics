#include "box2d-lite/Arbiter.h"
#include "box2d-lite/Body.h"
#include "box2d-lite/World.h"

Arbiter::Arbiter(Body* b1, Body* b2)
{
    if (b1 < b2)
    {
        body1 = b1;
        body2 = b2;
    }
    else
    {
        body1 = b2;
        body2 = b1;
    }

    numContacts = Collide(contacts, body1, body2);

    const CombineMode effectiveFrictionCombineMode = std::max(b1->shape.material.frictionCombineMode, b2->shape.material.frictionCombineMode);
    const CombineMode effectiveRestitutionCombineMode = std::max(b1->shape.material.restitutionCombineMode, b2->shape.material.restitutionCombineMode);
    switch (effectiveFrictionCombineMode)
    {
    case CombineMode::Average:
    {
        staticFriction = (body1->shape.material.staticFriction + body2->shape.material.staticFriction) * 0.5f;
        dynamicFriction = (body1->shape.material.dynamicFriction + body2->shape.material.dynamicFriction) * 0.5f;
        break;
    }
    case CombineMode::Minimum:
    {
        staticFriction = std::min(body1->shape.material.staticFriction, body2->shape.material.staticFriction);
        dynamicFriction = std::min(body1->shape.material.dynamicFriction, body2->shape.material.dynamicFriction);
        break;
    }
    case CombineMode::Multiply:
    {
        staticFriction = body1->shape.material.staticFriction * body2->shape.material.staticFriction;
        dynamicFriction = body1->shape.material.dynamicFriction * body2->shape.material.dynamicFriction;
        break;
    }
    case CombineMode::Maximum:
    {
        staticFriction = std::max(body1->shape.material.staticFriction, body2->shape.material.staticFriction);
        dynamicFriction = std::max(body1->shape.material.dynamicFriction, body2->shape.material.dynamicFriction);
        break;
    }
    default:
    {
        assert(false);
    }
    }
    switch (effectiveRestitutionCombineMode)
    {
    case CombineMode::Average:
    {
        restitution = (body1->shape.material.restitution + body2->shape.material.restitution) * 0.5f;
        break;
    }
    case CombineMode::Minimum:
    {
        restitution = std::min(body1->shape.material.restitution, body2->shape.material.restitution);
        break;
    }
    case CombineMode::Multiply:
    {
        restitution = body1->shape.material.restitution * body2->shape.material.restitution;
        break;
    }
    case CombineMode::Maximum:
    {
        restitution = std::max(body1->shape.material.restitution, body2->shape.material.restitution);
        break;
    }
    default:
    {
        assert(false);
    }
    }
}

void Arbiter::Update(Contact* newContacts, int numNewContacts)
{
    Contact mergedContacts[MAX_POINTS];

    for (int i = 0; i < numNewContacts; ++i)
    {
        Contact* cNew = newContacts + i;
        int k = -1;
        for (int j = 0; j < numContacts; ++j)
        {
            Contact* cOld = contacts + j;
            if (cNew->feature.value == cOld->feature.value)
            {
                k = j;
                break;
            }
        }

        if (k > -1)
        {
            Contact* c = mergedContacts + i;
            Contact* cOld = contacts + k;
            *c = *cNew;
            if (World::warmStarting)
            {
                c->Pn = cOld->Pn;
                c->Pt = cOld->Pt;
                c->Pnb = cOld->Pnb;
            }
            else
            {
                c->Pn = 0.0f;
                c->Pt = 0.0f;
                c->Pnb = 0.0f;
            }
        }
        else
        {
            mergedContacts[i] = newContacts[i];
        }
    }

    for (int i = 0; i < numNewContacts; ++i)
    {
        contacts[i] = mergedContacts[i];
    }

    numContacts = numNewContacts;
}

void Arbiter::PreStep(float inv_dt)
{
    const float k_allowedPenetration = 0.01f;
    float k_biasFactor = World::positionCorrection ? 0.1f : 0.0f;

    for (int i = 0; i < numContacts; ++i)
    {
        Contact* c = contacts + i;

        glm::vec3 r1 = c->position - body1->position;
        glm::vec3 r2 = c->position - body2->position;

        // Precompute normal mass, tangent mass, and bias.
        glm::vec3 rn1 = glm::cross(r1, c->normal);
        glm::vec3 rn2 = glm::cross(r2, c->normal);
        float kNormal = body1->invMass + body2->invMass;
        kNormal += glm::dot(rn1, body1->invI * rn1);
        kNormal += glm::dot(rn2, body2->invI * rn2);
        c->massNormal = 1.0f / kNormal;

        glm::vec3 tangent;
        if ((1.0f - std::abs(glm::dot(c->normal, glm::vec3(1.0f, 0.0f, 0.0f)))) < 1.0e-4f)
        {
            tangent = glm::normalize(glm::cross(c->normal, glm::vec3(0.0f, 1.0f, 0.0f)));
        }
        else
        {
            tangent = glm::normalize(glm::cross(c->normal, glm::vec3(1.0f, 0.0f, 0.0f)));
        }
        glm::vec3 rt1 = glm::cross(r1, tangent);
        glm::vec3 rt2 = glm::cross(r2, tangent);
        float kTangent = body1->invMass + body2->invMass;
        kTangent += glm::dot(rt1, body1->invI * rt1);
        kTangent += glm::dot(rt2, body2->invI * rt2);
        c->massTangent = 1.0f / kTangent;

        c->bias = -k_biasFactor * inv_dt * glm::min(0.0f, c->separation + k_allowedPenetration);

        glm::vec3 dv = body2->velocity + glm::cross(body2->angularVelocity, r2) - body1->velocity - glm::cross(body1->angularVelocity, r1);
        float vn = glm::dot(dv, c->normal);
        constexpr float velocityThreshold = 1.0f;
        if (vn < -velocityThreshold)
        {
            c->bias -= restitution * vn;
        }

        if (World::accumulateImpulses)
        {
            // Apply normal + friction impulse
            glm::vec3 P = c->Pn * c->normal + c->Pt * tangent;

            body1->velocity -= body1->invMass * P;
            body1->angularVelocity -= body1->invI * glm::cross(r1, P);

            body2->velocity += body2->invMass * P;
            body2->angularVelocity += body2->invI * glm::cross(r2, P);
        }
    }
}

void Arbiter::ApplyImpulse()
{
    for (int i = 0; i < numContacts; ++i)
    {
        Contact* c = contacts + i;
        c->r1 = c->position - body1->position;
        c->r2 = c->position - body2->position;

        // Relative velocity at contact
        glm::vec3 dv = body2->velocity + glm::cross(body2->angularVelocity, c->r2) - body1->velocity - glm::cross(body1->angularVelocity, c->r1);

        // Compute normal impulse
        float vn = glm::dot(dv, c->normal);

        float dPn = c->massNormal * (-vn + c->bias);

        if (World::accumulateImpulses)
        {
            // Clamp the accumulated impulse
            float Pn0 = c->Pn;
            c->Pn = glm::max(Pn0 + dPn, 0.0f);
            dPn = c->Pn - Pn0;
        }
        else
        {
            dPn = glm::max(dPn, 0.0f);
        }

        // Apply contact impulse
        glm::vec3 Pn = dPn * c->normal;

        body1->velocity -= body1->invMass * Pn;
        body1->angularVelocity -= body1->invI * glm::cross(c->r1, Pn);

        body2->velocity += body2->invMass * Pn;
        body2->angularVelocity += body2->invI * glm::cross(c->r2, Pn);

        // Relative velocity at contact
        dv = body2->velocity + glm::cross(body2->angularVelocity, c->r2) - body1->velocity - glm::cross(body1->angularVelocity, c->r1);

        glm::vec3 tangent;
        if ((1.0f - std::abs(glm::dot(c->normal, glm::vec3(1.0f, 0.0f, 0.0f)))) < 1.0e-4f)
        {
            tangent = glm::normalize(glm::cross(c->normal, glm::vec3(0.0f, 1.0f, 0.0f)));
        }
        else
        {
            tangent = glm::normalize(glm::cross(c->normal, glm::vec3(1.0f, 0.0f, 0.0f)));
        }
        float vt = glm::dot(dv, tangent);
        float dPt = c->massTangent * (-vt);

        constexpr float velocityThreshold = 1.0f;
        const float effectiveFriction = (std::abs(vt) < velocityThreshold) ? staticFriction : dynamicFriction;

        if (World::accumulateImpulses)
        {
            // Compute friction impulse
            float maxPt = effectiveFriction * c->Pn;

            // Clamp friction
            float oldTangentImpulse = c->Pt;
            c->Pt = glm::clamp(oldTangentImpulse + dPt, -maxPt, maxPt);
            dPt = c->Pt - oldTangentImpulse;
        }
        else
        {
            float maxPt = effectiveFriction * dPn;
            dPt = glm::clamp(dPt, -maxPt, maxPt);
        }

        // Apply contact impulse
        glm::vec3 Pt = dPt * tangent;

        body1->velocity -= body1->invMass * Pt;
        body1->angularVelocity -= body1->invI * glm::cross(c->r1, Pt);

        body2->velocity += body2->invMass * Pt;
        body2->angularVelocity += body2->invI * glm::cross(c->r2, Pt);
    }
}