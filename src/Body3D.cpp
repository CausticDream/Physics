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
	rotation.Identity();
	velocity.Set(0.0f, 0.0f, 0.0f);
	angularVelocity.Set(0.0f, 0.0f, 0.0f);
	force.Set(0.0f, 0.0f, 0.0f);
	torque.Set(0.0f, 0.0f, 0.0f);
	friction = 0.2f;
	mass = FLT_MAX;
	invMass = 0.0f;

	width.Set(1.0f, 1.0f, 1.0f);
	I.Identity();
	invI.Zero();
}

void Body3D::Set(const Vec3& w, float m)
{
	position.Set(0.0f, 0.0f, 0.0f);
	rotation.Identity();
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
		I.col1 = Vec3(mass * (width.y * width.y + width.z * width.z) / 12.0f, 0.0f, 0.0f);
		I.col2 = Vec3(0.0f, mass * (width.x * width.x + width.z * width.z) / 12.0f, 0.0f);
		I.col3 = Vec3(0.0f, 0.0f, mass * (width.x * width.x + width.y * width.y) / 12.0f);
		invI = I.Invert();
	}
	else
	{
		invMass = 0.0f;
		I.Identity();
		invI.Zero();
	}
}
