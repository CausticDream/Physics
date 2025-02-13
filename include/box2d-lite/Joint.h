#pragma once

#include <glm/glm.hpp>

struct Body;

struct Joint
{
    Joint()
    : body1(0)
    , body2(0)
    , P(0.0f, 0.0f, 0.0f)
    , biasFactor(0.2f)
    , softness(0.0f)
    {
    }

    void Set(Body* body1, Body* body2, const glm::vec3& anchor);

    void PreStep(float inv_dt);
    void ApplyImpulse();

    glm::mat3 M;
    glm::vec3 localAnchor1, localAnchor2;
    glm::vec3 r1, r2;
    glm::vec3 bias;
    glm::vec3 P;
    Body* body1;
    Body* body2;
    float biasFactor;
    float softness;
};