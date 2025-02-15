#pragma once

#include <glm/glm.hpp>
#include <Body.h>

constexpr size_t MAX_CONTACT_POINTS = 4;

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
    int feature;
};

size_t CollideBoxBox(Contact* contacts, Body* body1, Shape* shape1, Body* body2, Shape* shape2);
size_t CollideBoxSphere(Contact* contacts, Body* body1, Shape* shape1, Body* body2, Shape* shape2);
size_t CollideBoxCapsule(Contact* contacts, Body* body1, Shape* shape1, Body* body2, Shape* shape2);
size_t CollideSphereSphere(Contact* contacts, Body* body1, Shape* shape1, Body* body2, Shape* shape2);
size_t CollideSphereCapsule(Contact* contacts, Body* body1, Shape* shape1, Body* body2, Shape* shape2);
size_t CollideCapsuleCapsule(Contact* contacts, Body* body1, Shape* shape1, Body* body2, Shape* shape2);
size_t Collide(Contact* contacts, Body* body1, Shape* shape1, Body* body2, Shape* shape2);