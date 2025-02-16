#pragma once

#include <Collide.h>

struct Arbiter
{
    Arbiter(Shape* shape1, Shape* shape2);
    void Update(Contact* contacts, size_t contactCount, Contact* newContacts, size_t& newContactCount);
    void PreStep(float invElapsedTime);
    void ApplyImpulse();

    Body* m_body1;
    Body* m_body2;
    Contact m_contacts[g_maxContactPoints];
    size_t m_contactCount;
    float m_staticFriction;
    float m_dynamicFriction;
    float m_restitution;
};