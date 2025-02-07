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

#ifndef ARBITER3D_H
#define ARBITER3D_H

#include "MathUtils.h"

struct Body3D;

union Feature3DPair
{
	struct Edges
	{
		char inEdge1;
		char outEdge1;
		char inEdge2;
		char outEdge2;
	} e;
	int value;
};

struct Contact3D
{
	Contact3D() : Pn(0.0f), Pt(0.0f), Pnb(0.0f) {}

	Vec3 position;
	Vec3 normal;
	Vec3 r1, r2;
	float separation;
	float Pn;	// accumulated normal impulse
	float Pt;	// accumulated tangent impulse
	float Pnb;	// accumulated normal impulse for position bias
	float massNormal, massTangent;
	float bias;
	Feature3DPair feature;
};

struct Arbiter3DKey
{
	Arbiter3DKey(Body3D* b1, Body3D* b2)
	{
		if (b1 < b2)
		{
			body1 = b1; body2 = b2;
		}
		else
		{
			body1 = b2; body2 = b1;
		}
	}

	Body3D* body1;
	Body3D* body2;
};

struct Arbiter3D
{
	enum {MAX_POINTS = 2};

	Arbiter3D(Body3D* b1, Body3D* b2);

	void Update(Contact3D* contacts, int numContacts);

	void PreStep(float inv_dt);
	void ApplyImpulse();

	Contact3D contacts[MAX_POINTS];
	int numContacts;

	Body3D* body1;
	Body3D* body2;

	// Combined friction
	float friction;
};

// This is used by std::set
inline bool operator < (const Arbiter3DKey& a1, const Arbiter3DKey& a2)
{
	if (a1.body1 < a2.body1)
		return true;

	if (a1.body1 == a2.body1 && a1.body2 < a2.body2)
		return true;

	return false;
}

inline int Collide(Contact3D* contacts, Body3D* body1, Body3D* body2)
{
	// TODO
	return 0;
}

#endif
