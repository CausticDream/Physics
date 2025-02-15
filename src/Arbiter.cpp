#include "Arbiter.h"
#include "Body.h"
#include "World.h"

Arbiter::Arbiter(Shape* s1, Shape* s2)
{
    Shape* shape1;
    Shape* shape2;
    if (s1->GetUniqueID() < s2->GetUniqueID())
    {
        shape1 = s1;
        shape2 = s2;
    }
    else
    {
        shape1 = s2;
        shape2 = s1;
    }
    body1 = shape1->owner;
    body2 = shape2->owner;

    numContacts = CollideBoxBox(contacts, body1, static_cast<ShapeBox*>(shape1), body2, static_cast<ShapeBox*>(shape2));

    const CombineMode effectiveFrictionCombineMode = std::max(shape1->material->frictionCombineMode, shape2->material->frictionCombineMode);
    switch (effectiveFrictionCombineMode)
    {
    case CombineMode::Average:
    {
        staticFriction = (shape1->material->staticFriction + shape2->material->staticFriction) * 0.5f;
        dynamicFriction = (shape1->material->dynamicFriction + shape2->material->dynamicFriction) * 0.5f;
        break;
    }
    case CombineMode::Minimum:
    {
        staticFriction = std::min(shape1->material->staticFriction, shape2->material->staticFriction);
        dynamicFriction = std::min(shape1->material->dynamicFriction, shape2->material->dynamicFriction);
        break;
    }
    case CombineMode::Multiply:
    {
        staticFriction = shape1->material->staticFriction * shape2->material->staticFriction;
        dynamicFriction = shape1->material->dynamicFriction * shape2->material->dynamicFriction;
        break;
    }
    case CombineMode::Maximum:
    {
        staticFriction = std::max(shape1->material->staticFriction, shape2->material->staticFriction);
        dynamicFriction = std::max(shape1->material->dynamicFriction, shape2->material->dynamicFriction);
        break;
    }
    default:
    {
        assert(false);
    }
    }

    const CombineMode effectiveRestitutionCombineMode = std::max(shape1->material->restitutionCombineMode, shape2->material->restitutionCombineMode);
    switch (effectiveRestitutionCombineMode)
    {
    case CombineMode::Average:
    {
        restitution = (shape1->material->restitution + shape2->material->restitution) * 0.5f;
        break;
    }
    case CombineMode::Minimum:
    {
        restitution = std::min(shape1->material->restitution, shape2->material->restitution);
        break;
    }
    case CombineMode::Multiply:
    {
        restitution = shape1->material->restitution * shape2->material->restitution;
        break;
    }
    case CombineMode::Maximum:
    {
        restitution = std::max(shape1->material->restitution, shape2->material->restitution);
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
            if (cNew->feature == cOld->feature)
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
            // Apply normal + friction impulse.
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

        // Relative velocity at contact.
        glm::vec3 dv = body2->velocity + glm::cross(body2->angularVelocity, c->r2) - body1->velocity - glm::cross(body1->angularVelocity, c->r1);

        // Compute normal impulse.
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

        // Apply contact impulse.
        glm::vec3 Pn = dPn * c->normal;

        body1->velocity -= body1->invMass * Pn;
        body1->angularVelocity -= body1->invI * glm::cross(c->r1, Pn);

        body2->velocity += body2->invMass * Pn;
        body2->angularVelocity += body2->invI * glm::cross(c->r2, Pn);

        // Relative velocity at contact.
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
            // Compute friction impulse.
            float maxPt = effectiveFriction * c->Pn;

            // Clamp friction.
            float oldTangentImpulse = c->Pt;
            c->Pt = glm::clamp(oldTangentImpulse + dPt, -maxPt, maxPt);
            dPt = c->Pt - oldTangentImpulse;
        }
        else
        {
            float maxPt = effectiveFriction * dPn;
            dPt = glm::clamp(dPt, -maxPt, maxPt);
        }

        // Apply contact impulse.
        glm::vec3 Pt = dPt * tangent;

        body1->velocity -= body1->invMass * Pt;
        body1->angularVelocity -= body1->invI * glm::cross(c->r1, Pt);

        body2->velocity += body2->invMass * Pt;
        body2->angularVelocity += body2->invI * glm::cross(c->r2, Pt);
    }
}