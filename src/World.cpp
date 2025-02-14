#include "box2d-lite/World.h"
#include "box2d-lite/Body.h"
#include "box2d-lite/Joint.h"

bool World::accumulateImpulses = true;
bool World::warmStarting = true;
bool World::positionCorrection = true;

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
    for (int i = 0; i < (int)bodies.size(); ++i)
    {
        Body* bi = bodies[i];

        for (size_t s1 = 0; s1 < bi->shapes.size(); ++s1)
        {
            for (int j = i + 1; j < (int)bodies.size(); ++j)
            {
                Body* bj = bodies[j];

                if (bi->invMass == 0.0f && bj->invMass == 0.0f)
                {
                    continue;
                }

                for (size_t s2 = 0; s2 < bj->shapes.size(); ++s2)
                {
                    Arbiter newArb(bi->shapes[s1], bj->shapes[s2]);
                    ArbiterKey key(bi->shapes[s1], bj->shapes[s2]);

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
    for (int i = 0; i < (int)bodies.size(); ++i)
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

    for (int i = 0; i < (int)joints.size(); ++i)
    {
        joints[i]->PreStep(inv_dt);
    }

    // Perform iterations.
    for (int i = 0; i < iterations; ++i)
    {
        for (auto& arb : arbiters)
        {
            arb.second.ApplyImpulse();
        }

        for (int j = 0; j < (int)joints.size(); ++j)
        {
            joints[j]->ApplyImpulse();
        }
    }

    // Integrate Velocities.
    for (int i = 0; i < (int)bodies.size(); ++i)
    {
        Body* b = bodies[i];

        b->position += dt * b->velocity;
        b->rotation = glm::normalize(glm::quat(dt * b->angularVelocity) * b->rotation);

        b->force = glm::vec3(0.0f, 0.0f, 0.0f);
        b->torque = glm::vec3(0.0f, 0.0f, 0.0f);
    }
}