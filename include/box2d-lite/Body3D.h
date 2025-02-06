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

#ifndef BODY3D_H
#define BODY3D_H

#include "MathUtils.h"

struct Body3D
{
	Body3D();
	void Set(const Vec3& w, float m);

	void AddForce(const Vec3& f)
	{
		force += f;
	}

	Vec3 position;
	Quat rotation;

	Vec3 velocity;
	Vec3 angularVelocity;

	Vec3 force;
	Vec3 torque;

	Vec3 width;

	float friction;
	float mass, invMass;
	float I[3], invI[3];
};

#endif
