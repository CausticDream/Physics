/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erin Catto makes no representations about the suitability
* of this software for any purpose.
* It is provided "as is" without express or implied warranty.
*/

#include "box2d-lite/Joint3D.h"
#include "box2d-lite/Body3D.h"
#include "box2d-lite/World3D.h"

void Joint3D::Set(Body3D* b1, Body3D* b2, const Vec3& anchor)
{
	body1 = b1;
	body2 = b2;

	Mat33 Rot1 = body1->rotation.ToMatrix();
	Mat33 Rot2 = body2->rotation.ToMatrix();
	Mat33 Rot1T = Rot1.Transpose();
	Mat33 Rot2T = Rot2.Transpose();

	localAnchor1 = Rot1T * (anchor - body1->position);
	localAnchor2 = Rot2T * (anchor - body2->position);

	P.Set(0.0f, 0.0f, 0.0f);

	softness = 0.0f;
	biasFactor = 0.2f;
}

void Joint3D::PreStep(float inv_dt)
{
    // Pre-compute anchors, mass matrix, and bias.
    Mat33 Rot1 = body1->rotation.ToMatrix();
    Mat33 Rot2 = body2->rotation.ToMatrix();

    r1 = Rot1 * localAnchor1;
    r2 = Rot2 * localAnchor2;

    // Compute effective mass matrix (K)
    Mat33 K1;
    float diagonalValue = body1->invMass + body2->invMass;
    K1.col1 = Vec3(diagonalValue, 0.0f, 0.0f);
    K1.col2 = Vec3(0.0f, diagonalValue, 0.0f);
    K1.col3 = Vec3(0.0f, 0.0f, diagonalValue);

    // K2: Contribution from body1's angular inertia
    Mat33 K2;
    K2.col1 = body1->invI * Cross(r1, Vec3(1.0f, 0.0f, 0.0f));
    K2.col2 = body1->invI * Cross(r1, Vec3(0.0f, 1.0f, 0.0f));
    K2.col3 = body1->invI * Cross(r1, Vec3(0.0f, 0.0f, 1.0f));

    // K3: Contribution from body2's angular inertia
    Mat33 K3;
    K3.col1 = body2->invI * Cross(r2, Vec3(1.0f, 0.0f, 0.0f));
    K3.col2 = body2->invI * Cross(r2, Vec3(0.0f, 1.0f, 0.0f));
    K3.col3 = body2->invI * Cross(r2, Vec3(0.0f, 0.0f, 1.0f));

    // Combine contributions to form the full effective mass matrix
    Mat33 K = K1 + K2 + K3;

    // Add softness to the diagonal
    K.col1.x += softness;
    K.col2.y += softness;
    K.col3.z += softness;

    // Invert the effective mass matrix
    M = K.Invert();

    // Compute bias term for position correction
    Vec3 p1 = body1->position + r1;
    Vec3 p2 = body2->position + r2;
    Vec3 dp = p2 - p1;

    if (World3D::positionCorrection)
    {
        bias = -biasFactor * inv_dt * dp;
    }
    else
    {
        bias.Set(0.0f, 0.0f, 0.0f);
    }

    if (World3D::warmStarting)
    {
        // Apply accumulated impulse (warm start)
        body1->velocity -= body1->invMass * P;
        body1->angularVelocity -= body1->invI * Cross(r1, P);

        body2->velocity += body2->invMass * P;
        body2->angularVelocity += body2->invI * Cross(r2, P);
    }
    else
    {
        P.Set(0.0f, 0.0f, 0.0f);
    }
}

void Joint3D::ApplyImpulse()
{
    Vec3 dv = body2->velocity + Cross(body2->angularVelocity, r2) - body1->velocity - Cross(body1->angularVelocity, r1);
    Vec3 impulse = M * (bias - dv - softness * P);

    body1->velocity -= body1->invMass * impulse;
    body1->angularVelocity -= body1->invI * Cross(r1, impulse);

    body2->velocity += body2->invMass * impulse;
    body2->angularVelocity += body2->invI * Cross(r2, impulse);

    P += impulse;
}
