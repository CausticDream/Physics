#include "box2d-lite/World3D.h"
#include "box2d-lite/Body3D.h"
#include "box2d-lite/Joint3D.h"

using std::vector;
using std::map;
using std::pair;

typedef map<Arbiter3DKey, Arbiter3D>::iterator ArbIter;
typedef pair<Arbiter3DKey, Arbiter3D> ArbPair;

bool World3D::accumulateImpulses = true;
bool World3D::warmStarting = true;
bool World3D::positionCorrection = true;

void World3D::Add(Body3D* body)
{
	bodies.push_back(body);
}

void World3D::Add(Joint3D* joint)
{
	joints.push_back(joint);
}

void World3D::Clear()
{
	bodies.clear();
	joints.clear();
	arbiters.clear();
}

void World3D::BroadPhase()
{
	// O(n^2) broad-phase
	for (int i = 0; i < (int)bodies.size(); ++i)
	{
		Body3D* bi = bodies[i];

		for (int j = i + 1; j < (int)bodies.size(); ++j)
		{
			Body3D* bj = bodies[j];

			if (bi->invMass == 0.0f && bj->invMass == 0.0f)
				continue;

			Arbiter3D newArb(bi, bj);
			Arbiter3DKey key(bi, bj);

			if (newArb.numContacts > 0)
			{
				ArbIter iter = arbiters.find(key);
				if (iter == arbiters.end())
				{
					arbiters.insert(ArbPair(key, newArb));
				}
				else
				{
					iter->second.Update(newArb.contacts, newArb.numContacts);
				}
			}
			else
			{
				arbiters.erase(key);
			}
		}
	}
}

void World3D::Step(float dt)
{
	float inv_dt = dt > 0.0f ? 1.0f / dt : 0.0f;

	// Determine overlapping bodies and update contact points.
	BroadPhase();

	// Integrate forces.
	for (int i = 0; i < (int)bodies.size(); ++i)
	{
		Body3D* b = bodies[i];

		if (b->invMass == 0.0f)
			continue;

		b->velocity += dt * (gravity + b->invMass * b->force);
		b->angularVelocity += dt * (b->invI * b->torque);
	}

	// Perform pre-steps.
	for (ArbIter arb = arbiters.begin(); arb != arbiters.end(); ++arb)
	{
		arb->second.PreStep(inv_dt);
	}

	for (int i = 0; i < (int)joints.size(); ++i)
	{
		joints[i]->PreStep(inv_dt);
	}

	// Perform iterations
	for (int i = 0; i < iterations; ++i)
	{
		for (ArbIter arb = arbiters.begin(); arb != arbiters.end(); ++arb)
		{
			arb->second.ApplyImpulse();
		}

		for (int j = 0; j < (int)joints.size(); ++j)
		{
			joints[j]->ApplyImpulse();
		}
	}

	// Integrate Velocities
	for (int i = 0; i < (int)bodies.size(); ++i)
	{
		Body3D* b = bodies[i];

		b->position += dt * b->velocity;
		b->rotation = glm::normalize(glm::quat(dt * b->angularVelocity) * b->rotation);

		b->force = glm::vec3(0.0f, 0.0f, 0.0f);
		b->torque = glm::vec3(0.0f, 0.0f, 0.0f);
	}
}
