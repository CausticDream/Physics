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

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "MathUtils.h"

struct Body3D
{
	Body3D();
	void Set(const glm::vec3& w, float m);

	void AddForce(const glm::vec3& f)
	{
		force += f;
	}

	glm::vec3 position;
	glm::quat rotation;

	glm::vec3 velocity;
	glm::vec3 angularVelocity;

	glm::vec3 force;
	glm::vec3 torque;

	glm::vec3 width;

	float friction;
	float mass, invMass;
	glm::mat3 I, invI;
};

#endif
