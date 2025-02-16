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

World::World(glm::vec3 gravity, uint32_t iterations)
: m_gravity(gravity)
, m_iterations(iterations)
{
}

void World::Clear()
{
    m_bodies.clear();
    m_joints.clear();
    m_arbiters.clear();
}

void World::Add(Body* body)
{
    m_bodies.push_back(body);
}

void World::Add(Joint* joint)
{
    m_joints.push_back(joint);
}

void World::Remove(Body* body)
{
    m_bodies.erase(std::find(m_bodies.begin(), m_bodies.end(), body));

    for (size_t i = 0; i < m_bodies.size(); ++i)
    {
        Body* bi = m_bodies[i];
        for (size_t s1 = 0; s1 < bi->m_shapes.size(); ++s1)
        {
            for (size_t j = i + 1; j < m_bodies.size(); ++j)
            {
                Body* bj = m_bodies[j];
                for (size_t s2 = 0; s2 < bj->m_shapes.size(); ++s2)
                {
                    const uint64_t key = ComputeArbiterKey(bi->m_shapes[s1], bj->m_shapes[s2]);
                    m_arbiters.erase(key);
                }
            }
        }
    }
}

void World::Remove(Joint* joint)
{
    m_joints.erase(std::find(m_joints.begin(), m_joints.end(), joint));
}

void World::BroadPhase()
{
    for (size_t i = 0; i < m_bodies.size(); ++i)
    {
        Body* bi = m_bodies[i];
        for (size_t s1 = 0; s1 < bi->m_shapes.size(); ++s1)
        {
            for (size_t j = i + 1; j < m_bodies.size(); ++j)
            {
                Body* bj = m_bodies[j];

                if (bi->m_invMass == 0.0f && bj->m_invMass == 0.0f)
                {
                    continue;
                }

                for (size_t s2 = 0; s2 < bj->m_shapes.size(); ++s2)
                {
                    Arbiter newArb(bi->m_shapes[s1], bj->m_shapes[s2]);
                    const uint64_t key = ComputeArbiterKey(bi->m_shapes[s1], bj->m_shapes[s2]);

                    if (newArb.m_contactCount > 0)
                    {
                        const auto iter = m_arbiters.find(key);
                        if (iter == m_arbiters.end())
                        {
                            m_arbiters.insert({key, newArb});
                        }
                        else
                        {
                            iter->second.Update(newArb.m_contacts, newArb.m_contactCount);
                        }
                    }
                    else
                    {
                        m_arbiters.erase(key);
                    }
                }
            }
        }
    }
}

void World::Step(float elapsedTime)
{
    // Determine overlapping bodies and update contact points.
    BroadPhase();

    // Integrate forces.
    for (size_t i = 0; i < m_bodies.size(); ++i)
    {
        Body* b = m_bodies[i];

        if (b->m_invMass == 0.0f)
        {
            continue;
        }

        b->m_velocity += elapsedTime * (m_gravity + b->m_invMass * b->m_force);
        b->m_angularVelocity += elapsedTime * (b->m_invI * b->m_torque);
    }

    // Perform pre-steps.
    float invElapsedTime = (elapsedTime > 0.0f) ? 1.0f / elapsedTime : 0.0f;

    for (auto& arb : m_arbiters)
    {
        arb.second.PreStep(invElapsedTime);
    }

    for (size_t i = 0; i < m_joints.size(); ++i)
    {
        m_joints[i]->PreStep(invElapsedTime);
    }

    // Perform iterations.
    for (uint32_t i = 0; i < m_iterations; ++i)
    {
        for (auto& arb : m_arbiters)
        {
            arb.second.ApplyImpulse();
        }

        for (size_t j = 0; j < m_joints.size(); ++j)
        {
            m_joints[j]->ApplyImpulse();
        }
    }

    // Integrate Velocities.
    for (size_t i = 0; i < m_bodies.size(); ++i)
    {
        Body* b = m_bodies[i];

        b->m_position += elapsedTime * b->m_velocity;
        b->m_rotation = glm::normalize(glm::quat(elapsedTime * b->m_angularVelocity) * b->m_rotation);

        b->m_force = glm::vec3(0.0f, 0.0f, 0.0f);
        b->m_torque = glm::vec3(0.0f, 0.0f, 0.0f);
    }
}