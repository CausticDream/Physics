#pragma once

#include <Collide.h>

struct ArbiterKey
{
    ArbiterKey(Shape* s1, Shape* s2)
    {
        if (s1->GetUniqueID() < s2->GetUniqueID())
        {
            shape1 = s1;
            shape2 = s2;
        }
        else
        {
            shape1 = s2;
            shape2 = s1;
        }
    }

    bool operator==(const ArbiterKey& other) const
    {
        return (shape1->GetUniqueID() == other.shape1->GetUniqueID()) && (shape2->GetUniqueID() == other.shape2->GetUniqueID());
    }

    Shape* shape1;
    Shape* shape2;
};

namespace std
{
    template<>
    struct hash<ArbiterKey>
    {
        size_t operator()(const ArbiterKey& s) const
        {
            return static_cast<size_t>((static_cast<uint64_t>(s.shape1->GetUniqueID()) << 32) | s.shape2->GetUniqueID());
        }
    };
}

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