#pragma once

#include "Arbiter.h"
#include <glm/glm.hpp>
#include <map>
#include <vector>

struct Body;
struct Joint;

struct World
{
    World(glm::vec3 gravity, int iterations)
    : gravity(gravity)
    , iterations(iterations)
    {
    }

    void Add(Body* body);
    void Add(Joint* joint);
    void Clear();

    void Step(float dt);

    void BroadPhase();

    std::vector<Body*> bodies;
    std::vector<Joint*> joints;
    std::map<ArbiterKey, Arbiter> arbiters;
    glm::vec3 gravity;
    int iterations;
    static bool accumulateImpulses;
    static bool warmStarting;
    static bool positionCorrection;
};