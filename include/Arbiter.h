#pragma once

#include <Collide.h>

struct Arbiter
{
    Arbiter(Shape* s1, Shape* s2);
    void Update(Contact* contacts, size_t numContacts);
    void PreStep(float inv_dt);
    void ApplyImpulse();

    Body* body1;
    Body* body2;
    Contact contacts[MAX_CONTACT_POINTS];
    size_t numContacts;
    float staticFriction;
    float dynamicFriction;
    float restitution;
};