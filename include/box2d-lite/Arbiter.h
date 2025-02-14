#pragma once

#include <glm/glm.hpp>

struct Body;
struct Shape;
struct ShapeBox;

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
    ArbiterKey(Shape* s1, Shape* s2)
    {
        if (s1 < s2)
        {
            shape1 = s1;
            shape2 = s2;
        }
        else
        {
            shape1 = s2;
            shape2 = s2;
        }
    }

    Shape* shape1;
    Shape* shape2;
};

struct Arbiter
{
    enum
    {
        MAX_POINTS = 4
    };

    Arbiter(Shape* s1, Shape* s2);

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
    if (a1.shape1 < a2.shape1)
    {
        return true;
    }

    if (a1.shape1 == a2.shape1 && a1.shape2 < a2.shape2)
    {
        return true;
    }

    return false;
}

int CollideBoxBox(Contact* contacts, Body* body1, ShapeBox* shape1, Body* body2, ShapeBox* shape2);