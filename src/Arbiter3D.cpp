/*
* Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erin Catto makes no representations about the suitability
* of this software for any purpose.
* It is provided "as is" without express or implied warranty.
*/

#include "box2d-lite/Arbiter3D.h"
#include "box2d-lite/Body3D.h"
#include "box2d-lite/World3D.h"

Arbiter3D::Arbiter3D(Body3D* b1, Body3D* b2)
{
	if (b1 < b2)
	{
		body1 = b1;
		body2 = b2;
	}
	else
	{
		body1 = b2;
		body2 = b1;
	}

	numContacts = Collide(contacts, body1, body2);

	friction = sqrtf(body1->friction * body2->friction);
}

void Arbiter3D::Update(Contact3D* newContacts, int numNewContacts)
{
	Contact3D mergedContacts[2];

	for (int i = 0; i < numNewContacts; ++i)
	{
		Contact3D* cNew = newContacts + i;
		int k = -1;
		for (int j = 0; j < numContacts; ++j)
		{
			Contact3D* cOld = contacts + j;
			if (cNew->feature.value == cOld->feature.value)
			{
				k = j;
				break;
			}
		}

		if (k > -1)
		{
			Contact3D* c = mergedContacts + i;
			Contact3D* cOld = contacts + k;
			*c = *cNew;
			if (World3D::warmStarting)
			{
				c->Pn = cOld->Pn;
				c->Pt = cOld->Pt;
				c->Pnb = cOld->Pnb;
			}
			else
			{
				c->Pn = 0.0f;
				c->Pt = 0.0f;
				c->Pnb = 0.0f;
			}
		}
		else
		{
			mergedContacts[i] = newContacts[i];
		}
	}

	for (int i = 0; i < numNewContacts; ++i)
		contacts[i] = mergedContacts[i];

	numContacts = numNewContacts;
}


void Arbiter3D::PreStep(float inv_dt)
{
	const float k_allowedPenetration = 0.01f;
	float k_biasFactor = World3D::positionCorrection ? 0.2f : 0.0f;

	for (int i = 0; i < numContacts; ++i)
	{
		Contact3D* c = contacts + i;

		glm::vec3 r1 = c->position - body1->position;
		glm::vec3 r2 = c->position - body2->position;

		// Precompute normal mass, tangent mass, and bias.
		float rn1 = glm::dot(r1, c->normal);
		float rn2 = glm::dot(r2, c->normal);
		float kNormal = body1->invMass + body2->invMass;
		kNormal += glm::dot(glm::cross(r1, c->normal), body1->invI * glm::cross(r1, c->normal));
		kNormal += glm::dot(glm::cross(r2, c->normal), body2->invI * glm::cross(r2, c->normal));
		c->massNormal = 1.0f / kNormal;

		glm::vec3 tangent = glm::normalize(glm::cross(c->normal, glm::vec3(1.0f, 0.0f, 0.0f))); // Arbitrary perpendicular vector
		float rt1 = glm::dot(r1, tangent);
		float rt2 = glm::dot(r2, tangent);
		float kTangent = body1->invMass + body2->invMass;
		kTangent += glm::dot(glm::cross(r1, tangent), body1->invI * glm::cross(r1, tangent));
		kTangent += glm::dot(glm::cross(r2, tangent), body2->invI * glm::cross(r2, tangent));
		c->massTangent = 1.0f / kTangent;

		c->bias = -k_biasFactor * inv_dt * Min(0.0f, c->separation + k_allowedPenetration);

		if (World3D::accumulateImpulses)
		{
			// Apply normal + friction impulse
			glm::vec3 P = c->Pn * c->normal + c->Pt * tangent;

			body1->velocity -= body1->invMass * P;
			body1->angularVelocity -= body1->invI * glm::cross(r1, P);

			body2->velocity += body2->invMass * P;
			body2->angularVelocity += body2->invI * glm::cross(r2, P);
		}
	}
}

void Arbiter3D::ApplyImpulse()
{
	Body3D* b1 = body1;
	Body3D* b2 = body2;

	for (int i = 0; i < numContacts; ++i)
	{
		Contact3D* c = contacts + i;

		glm::vec3 r1 = c->position - b1->position;
		glm::vec3 r2 = c->position - b2->position;

		// Relative velocity at contact
		glm::vec3 dv = b2->velocity + glm::cross(b2->angularVelocity, r2) - b1->velocity - glm::cross(b1->angularVelocity, r1);

		// Compute normal impulse
		float vn = glm::dot(dv, c->normal);

		float dPn = c->massNormal * (-vn + c->bias);

		if (World3D::accumulateImpulses)
		{
			// Clamp the accumulated impulse
			float Pn0 = c->Pn;
			c->Pn = Max(Pn0 + dPn, 0.0f);
			dPn = c->Pn - Pn0;
		}
		else
		{
			dPn = Max(dPn, 0.0f);
		}

		// Apply contact impulse
		glm::vec3 Pn = dPn * c->normal;

		b1->velocity -= b1->invMass * Pn;
		b1->angularVelocity -= b1->invI * glm::cross(r1, Pn);

		b2->velocity += b2->invMass * Pn;
		b2->angularVelocity += b2->invI * glm::cross(r2, Pn);

		// Relative velocity at contact
		dv = b2->velocity + glm::cross(b2->angularVelocity, r2) - b1->velocity - glm::cross(b1->angularVelocity, r1);

		glm::vec3 tangent = glm::normalize(glm::cross(glm::cross(c->normal, glm::vec3(1.0f, 0.0f, 0.0f)), c->normal));
		float vt = glm::dot(dv, tangent);
		float dPt = c->massTangent * (-vt);

		if (World3D::accumulateImpulses)
		{
			// Compute friction impulse
			float maxPt = friction * c->Pn;

			// Clamp friction
			float oldTangentImpulse = c->Pt;
			c->Pt = Clamp(oldTangentImpulse + dPt, -maxPt, maxPt);
			dPt = c->Pt - oldTangentImpulse;
		}
		else
		{
			float maxPt = friction * dPn;
			dPt = Clamp(dPt, -maxPt, maxPt);
		}

		// Apply tangent impulse
		glm::vec3 Pt = dPt * tangent;

		b1->velocity -= b1->invMass * Pt;
		b1->angularVelocity -= b1->invI * glm::cross(r1, Pt);

		b2->velocity += b2->invMass * Pt;
		b2->angularVelocity += b2->invI * glm::cross(r2, Pt);
	}
}
