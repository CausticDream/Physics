#include "box2d-lite/Joint3D.h"
#include "box2d-lite/Body3D.h"
#include "box2d-lite/World3D.h"

void Joint3D::Set(Body3D* b1, Body3D* b2, const glm::vec3& anchor)
{
    body1 = b1;
    body2 = b2;

    localAnchor1 = glm::conjugate(body1->rotation) * (anchor - body1->position);
    localAnchor2 = glm::conjugate(body2->rotation) * (anchor - body2->position);

    P = glm::vec3(0.0f, 0.0f, 0.0f);

    softness = 0.0f;
    biasFactor = 0.2f;
}

void Joint3D::PreStep(float inv_dt)
{
    // Pre-compute anchors, mass matrix, and bias.
    r1 = body1->rotation * localAnchor1;
    r2 = body2->rotation * localAnchor2;

    // deltaV = deltaV0 + K * impulse
    // invM = [(1/m1 + 1/m2) * eye(3) - skew(r1) * invI1 * skew(r1)^T - skew(r2) * invI2 * skew(r2)^T]
    glm::mat3 K1 = glm::mat3(0.0f);
    K1[0].x = body1->invMass + body2->invMass;
    K1[1].y = body1->invMass + body2->invMass;
    K1[2].z = body1->invMass + body2->invMass;

    glm::mat3 skewR1 = glm::mat3(
        0.0f, -r1.z, r1.y,
        r1.z, 0.0f, -r1.x,
        -r1.y, r1.x, 0.0f
    );
    glm::mat3 K2 = skewR1 * body1->invI * glm::transpose(skewR1);

    glm::mat3 skewR2 = glm::mat3(
        0.0f, -r2.z, r2.y,
        r2.z, 0.0f, -r2.x,
        -r2.y, r2.x, 0.0f
    );
    glm::mat3 K3 = skewR2 * body2->invI * glm::transpose(skewR2);

    glm::mat3 K = K1 + K2 + K3;
    K[0].x += softness;
    K[1].y += softness;
    K[2].z += softness;

    M = glm::inverse(K);

    glm::vec3 p1 = body1->position + r1;
    glm::vec3 p2 = body2->position + r2;
    glm::vec3 dp = p2 - p1;

    if (World3D::positionCorrection)
    {
        bias = -biasFactor * inv_dt * dp;
    }
    else
    {
        bias = glm::vec3(0.0f, 0.0f, 0.0f);
    }

    if (World3D::warmStarting)
    {
        // Apply accumulated impulse.
        body1->velocity -= body1->invMass * P;
        body1->angularVelocity -= body1->invI * glm::cross(r1, P);

        body2->velocity += body2->invMass * P;
        body2->angularVelocity += body2->invI * glm::cross(r2, P);
    }
    else
    {
        P = glm::vec3(0.0f, 0.0f, 0.0f);
    }
}

void Joint3D::ApplyImpulse()
{
    glm::vec3 dv = body2->velocity + glm::cross(body2->angularVelocity, r2) - body1->velocity - glm::cross(body1->angularVelocity, r1);
    glm::vec3 impulse = M * (bias - dv - softness * P);

    body1->velocity -= body1->invMass * impulse;
    body1->angularVelocity -= body1->invI * glm::cross(r1, impulse);

    body2->velocity += body2->invMass * impulse;
    body2->angularVelocity += body2->invI * glm::cross(r2, impulse);

    P += impulse;
}
