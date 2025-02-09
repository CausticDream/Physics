#pragma once

#include "Arbiter3D.h"
#include <glm/glm.hpp>
#include <map>
#include <vector>

struct Body3D;
struct Joint3D;

struct World3D
{
    World3D(glm::vec3 gravity, int iterations)
    : gravity(gravity)
    , iterations(iterations)
    {
    }

    void Add(Body3D* body);
    void Add(Joint3D* joint);
    void Clear();

    void Step(float dt);

    void BroadPhase();

    std::vector<Body3D*> bodies;
    std::vector<Joint3D*> joints;
    std::map<Arbiter3DKey, Arbiter3D> arbiters;
    glm::vec3 gravity;
    int iterations;
    static bool accumulateImpulses;
    static bool warmStarting;
    static bool positionCorrection;
};