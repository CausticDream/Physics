#include "box2d-lite/Joint3D.h"
#include "box2d-lite/Body3D.h"
#include "box2d-lite/World3D.h"

void Joint3D::Set(Body3D* b1, Body3D* b2, const glm::vec3& anchor)
{
	body1 = b1;
	body2 = b2;

	glm::mat3 Rot1 = glm::mat3_cast(body1->rotation);
    glm::mat3 Rot2 = glm::mat3_cast(body2->rotation);
    glm::mat3 Rot1T = glm::transpose(Rot1);
    glm::mat3 Rot2T = glm::transpose(Rot2);

	localAnchor1 = Rot1T * (anchor - body1->position);
	localAnchor2 = Rot2T * (anchor - body2->position);

	P = glm::vec3(0.0f, 0.0f, 0.0f);

	softness = 0.0f;
	biasFactor = 0.2f;
}

void Joint3D::PreStep(float inv_dt)
{
    // Pre-compute anchors, mass matrix, and bias.
    glm::mat3 Rot1 = glm::mat3_cast(body1->rotation);
    glm::mat3 Rot2 = glm::mat3_cast(body2->rotation);

    r1 = Rot1 * localAnchor1;
    r2 = Rot2 * localAnchor2;

    // Compute effective mass matrix (K)
    float sumInvMasses = body1->invMass + body2->invMass;
    glm::mat3 K1 = glm::mat3(sumInvMasses);

    // K2: Contribution from body1's angular inertia
    glm::mat3 skewR1(
        glm::vec3(0.0f, -r1.z, r1.y),
        glm::vec3(r1.z, 0.0f, -r1.x),
        glm::vec3(-r1.y, r1.x, 0.0f)
    );
    glm::mat3 K2 = -skewR1 * body1->invI * glm::transpose(skewR1);

    // K3: Contribution from body2's angular inertia
    glm::mat3 skewR2(
        glm::vec3(0.0f, -r2.z, r2.y),
        glm::vec3(r2.z, 0.0f, -r2.x),
        glm::vec3(-r2.y, r2.x, 0.0f)
    );
    glm::mat3 K3 = -skewR2 * body2->invI * glm::transpose(skewR2);

    // Combine contributions to form the full effective mass matrix
    glm::mat3 K = K1 + K2 + K3;

    // Add softness to the diagonal
    K[0][0] += softness;
    K[1][1] += softness;
    K[2][2] += softness;

    // Invert the effective mass matrix
    M = glm::inverse(K);

    // Compute bias term for position correction
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
        // Apply accumulated impulse (warm start)
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
