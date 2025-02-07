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

#include "box2d-lite/Body3D.h"

Body3D::Body3D()
{
	position.Set(0.0f, 0.0f, 0.0f);
	rotation.Set(0.0f, 0.0f, 0.0f, 1.0f);
	velocity.Set(0.0f, 0.0f, 0.0f);
	angularVelocity.Set(0.0f, 0.0f, 0.0f);
	force.Set(0.0f, 0.0f, 0.0f);
	torque.Set(0.0f, 0.0f, 0.0f);
	friction = 0.2f;

	width.Set(1.0f, 1.0f, 1.0f);
	mass = FLT_MAX;
	invMass = 0.0f;
	I.x = FLT_MAX;
	invI.x = 0.0f;
	I.y = FLT_MAX;
	invI.y = 0.0f;
	I.z = FLT_MAX;
	invI.z = 0.0f;
}

void Body3D::Set(const Vec3& w, float m)
{
	position.Set(0.0f, 0.0f, 0.0f);
	rotation.Set(0.0f, 0.0f, 0.0f, 1.0f);
	velocity.Set(0.0f, 0.0f, 0.0f);
	angularVelocity.Set(0.0f, 0.0f, 0.0f);
	force.Set(0.0f, 0.0f, 0.0f);
	torque.Set(0.0f, 0.0f, 0.0f);
	friction = 0.2f;

	width = w;
	mass = m;

	if (mass < FLT_MAX)
	{
		invMass = 1.0f / mass;
		I.x = mass * (width.y * width.y + width.z * width.z) / 12.0f;
		I.y = mass * (width.x * width.x + width.z * width.z) / 12.0f;
		I.z = mass * (width.x * width.x + width.y * width.y) / 12.0f;
		invI.x = 1.0f / I.x;
		invI.y = 1.0f / I.y;
		invI.z = 1.0f / I.z;
	}
	else
	{
		invMass = 0.0f;
		I.x = FLT_MAX;
		invI.x = 0.0f;
		I.y = FLT_MAX;
		invI.y = 0.0f;
		I.z = FLT_MAX;
		invI.z = 0.0f;
	}
}
