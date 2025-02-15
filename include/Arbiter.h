#pragma once

#include <Collide.h>

struct Arbiter
{
    Arbiter(Shape* s1, Shape* s2);

    void Update(Contact* contacts, size_t numContacts);

    void PreStep(float inv_dt);
    void ApplyImpulse();

    Contact contacts[MAX_CONTACT_POINTS];
    size_t numContacts;

    Body* body1;
    Body* body2;

    float staticFriction;
    float dynamicFriction;
    float restitution;
};