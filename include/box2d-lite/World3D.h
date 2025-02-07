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

#ifndef WORLD3D_H
#define WORLD3D_H

#include <vector>
#include <map>
#include "MathUtils.h"
#include "Arbiter3D.h"

struct Body3D;
struct Joint3D;

struct World3D
{
	World3D(Vec3 gravity, int iterations) : gravity(gravity), iterations(iterations) {}

	void Add(Body3D* body);
	void Add(Joint3D* joint);
	void Clear();

	void Step(float dt);

	void BroadPhase();

	std::vector<Body3D*> bodies;
	std::vector<Joint3D*> joints;
	std::map<Arbiter3DKey, Arbiter3D> arbiters;
	Vec3 gravity;
	int iterations;
	static bool accumulateImpulses;
	static bool warmStarting;
	static bool positionCorrection;
};

#endif
