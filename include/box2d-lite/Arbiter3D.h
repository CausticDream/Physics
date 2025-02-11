#pragma once

#include <glm/glm.hpp>

struct Body3D;

union Feature3DPair
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

struct Contact3D
{
    Contact3D()
    : Pn(0.0f)
    , Pt(0.0f)
    , Pnb(0.0f)
    {
    }

    glm::vec3 position;
    glm::vec3 normal;
    glm::vec3 r1, r2;
    float separation;
    float Pn; // accumulated normal impulse
    float Pt; // accumulated tangent impulse
    float Pnb; // accumulated normal impulse for position bias
    float massNormal, massTangent;
    float bias;
    Feature3DPair feature;
};

struct Arbiter3DKey
{
    Arbiter3DKey(Body3D* b1, Body3D* b2)
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

    Body3D* body1;
    Body3D* body2;
};

struct Arbiter3D
{
    enum
    {
        MAX_POINTS = 4
    };

    Arbiter3D(Body3D* b1, Body3D* b2);

    void Update(Contact3D* contacts, int numContacts);

    void PreStep(float inv_dt);
    void ApplyImpulse();

    Contact3D contacts[MAX_POINTS];
    int numContacts;

    Body3D* body1;
    Body3D* body2;

    // Combined friction and restitution
    float staticFriction;
    float dynamicFriction;
    float restitution;
};

// This is used by std::set
inline bool operator<(const Arbiter3DKey& a1, const Arbiter3DKey& a2)
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

int Collide3D(Contact3D* contacts, Body3D* body1, Body3D* body2);