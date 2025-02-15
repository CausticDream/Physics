#pragma once

#include "Arbiter.h"
#include <glm/glm.hpp>
#include <unordered_map>
#include <vector>

struct Body;
struct Joint;

struct World
{
    World(glm::vec3 gravity, uint32_t iterations)
    : gravity(gravity)
    , iterations(iterations)
    {
    }

    void Add(Body* body);
    void Add(Joint* joint);
    void Clear();
    void Step(float dt);
    void BroadPhase();

    glm::vec3 gravity;
    uint32_t iterations;
    std::vector<Body*> bodies;
    std::vector<Joint*> joints;
    std::unordered_map<uint64_t, Arbiter> arbiters;
};