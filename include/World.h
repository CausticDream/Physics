#pragma once

#include "Arbiter.h"
#include <glm/glm.hpp>
#include <unordered_map>
#include <vector>

struct Body;
struct Joint;

struct World
{
    World(glm::vec3 gravity, uint32_t iterations);
    void Clear();
    void Add(Body* body);
    void Remove(Body* body);
    void Add(Joint* joint);
    void Remove(Joint* joint);
    void Step(float elapsedTime);
    void BroadPhase();

    glm::vec3 m_gravity;
    uint32_t m_iterations;
    std::vector<Body*> m_bodies;
    std::vector<Joint*> m_joints;
    std::unordered_map<uint64_t, Arbiter> m_arbiters;
};