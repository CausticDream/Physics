#include "World.h"
#include "Body.h"
#include "Joint.h"

uint64_t ComputeArbiterKey(Shape* s1, Shape* s2)
{
    if (s1->GetUniqueID() < s2->GetUniqueID())
    {
        return static_cast<size_t>((static_cast<uint64_t>(s1->GetUniqueID()) << 32) | s2->GetUniqueID());
    }
    else
    {
        return static_cast<size_t>((static_cast<uint64_t>(s2->GetUniqueID()) << 32) | s1->GetUniqueID());
    }
}

void World::Add(Body* body)
{
    bodies.push_back(body);
}

void World::Add(Joint* joint)
{
    joints.push_back(joint);
}

void World::Clear()
{
    bodies.clear();
    joints.clear();
    arbiters.clear();
}

void World::BroadPhase()
{
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        Body* bi = bodies[i];
        for (size_t s1 = 0; s1 < bi->shapes.size(); ++s1)
        {
            for (size_t j = i + 1; j < bodies.size(); ++j)
            {
                Body* bj = bodies[j];

                if (bi->invMass == 0.0f && bj->invMass == 0.0f)
                {
                    continue;
                }

                for (size_t s2 = 0; s2 < bj->shapes.size(); ++s2)
                {
                    Arbiter newArb(bi->shapes[s1], bj->shapes[s2]);
                    const uint64_t key = ComputeArbiterKey(bi->shapes[s1], bj->shapes[s2]);

                    if (newArb.numContacts > 0)
                    {
                        const auto iter = arbiters.find(key);
                        if (iter == arbiters.end())
                        {
                            arbiters.insert({key, newArb});
                        }
                        else
                        {
                            iter->second.Update(newArb.contacts, newArb.numContacts);
                        }
                    }
                    else
                    {
                        arbiters.erase(key);
                    }
                }
            }
        }
    }
}

void World::Step(float dt)
{
    float inv_dt = dt > 0.0f ? 1.0f / dt : 0.0f;

    // Determine overlapping bodies and update contact points.
    BroadPhase();

    // Integrate forces.
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        Body* b = bodies[i];

        if (b->invMass == 0.0f)
        {
            continue;
        }

        b->velocity += dt * (gravity + b->invMass * b->force);
        b->angularVelocity += dt * (b->invI * b->torque);
    }

    // Perform pre-steps.
    for (auto& arb : arbiters)
    {
        arb.second.PreStep(inv_dt);
    }

    for (size_t i = 0; i < joints.size(); ++i)
    {
        joints[i]->PreStep(inv_dt);
    }

    // Perform iterations.
    for (uint32_t i = 0; i < iterations; ++i)
    {
        for (auto& arb : arbiters)
        {
            arb.second.ApplyImpulse();
        }

        for (size_t j = 0; j < joints.size(); ++j)
        {
            joints[j]->ApplyImpulse();
        }
    }

    // Integrate Velocities.
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        Body* b = bodies[i];

        b->position += dt * b->velocity;
        b->rotation = glm::normalize(glm::quat(dt * b->angularVelocity) * b->rotation);

        b->force = glm::vec3(0.0f, 0.0f, 0.0f);
        b->torque = glm::vec3(0.0f, 0.0f, 0.0f);
    }
}