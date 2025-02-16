#pragma once

#include "Arbiter.h"
#include <glm/glm.hpp>
#include <unordered_map>
#include <vector>

struct Body;
struct Joint;

struct CollisionResult
{
    glm::vec3 m_position;
    glm::vec3 m_normal;
    glm::vec3 m_impulse;
    float m_separation;
};

struct WorldListener
{
    virtual void OnCollision(Body* body1, Body* body2, CollisionResult* collisionResults, size_t collisionCount) = 0;
};

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
    std::vector<WorldListener*> m_worldListeners;
};