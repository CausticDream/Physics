#pragma once

#include <glm/glm.hpp>

struct Body;

union FeaturePair
{
    struct Faces
    {
        char inFace1;
        char outFace1;
        char inFace2;
        char outFace2;
    } f;
    int value;
};

struct Contact
{
    Contact()
    : Pn(0.0f)
    , Pt(0.0f)
    , Pnb(0.0f)
    {
    }

    glm::vec3 position;
    glm::vec3 normal;
    glm::vec3 r1, r2;
    float separation;
    float Pn;
    float Pt;
    float Pnb;
    float massNormal, massTangent;
    float bias;
    FeaturePair feature;
};

struct ArbiterKey
{
    ArbiterKey(Body* b1, Body* b2)
    {
        if (b1 < b2)
        {
            body1 = b1;
            body2 = b2;
        }
        else
        {
            body1 = b2;
            body2 = b1;
        }
    }

    Body* body1;
    Body* body2;
};

struct Arbiter
{
    enum
    {
        MAX_POINTS = 4
    };

    Arbiter(Body* b1, Body* b2);

    void Update(Contact* contacts, int numContacts);

    void PreStep(float inv_dt);
    void ApplyImpulse();

    Contact contacts[MAX_POINTS];
    int numContacts;

    Body* body1;
    Body* body2;

    float staticFriction;
    float dynamicFriction;
    float restitution;
};

inline bool operator<(const ArbiterKey& a1, const ArbiterKey& a2)
{
    if (a1.body1 < a2.body1)
    {
        return true;
    }

    if (a1.body1 == a2.body1 && a1.body2 < a2.body2)
    {
        return true;
    }

    return false;
}

int Collide(Contact* contacts, Body* body1, Body* body2);