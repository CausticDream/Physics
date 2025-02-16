#pragma once

#include <glm/glm.hpp>
#include <Body.h>

constexpr size_t MAX_CONTACT_POINTS = 4;

struct Contact
{
    Contact()
    : separation(0.0f)
    , Pn(0.0f)
    , Pt(0.0f)
    , Pnb(0.0f)
    , massNormal(0.0f)
    , massTangent(0.0f)
    , bias(0.0f)
    , feature(0)
    {
    }

    glm::vec3 position;
    glm::vec3 normal;
    glm::vec3 r1;
    glm::vec3 r2;
    float separation;
    float Pn;
    float Pt;
    float Pnb;
    float massNormal;
    float massTangent;
    float bias;
    uint32_t feature;
};

size_t CollideBoxBox(Contact* contacts, glm::vec3 positionShape1, glm::quat rotationShape1, Shape* shape1, glm::vec3 positionShape2, glm::quat rotationShape2, Shape* shape2);
size_t CollideBoxSphere(Contact* contacts, glm::vec3 positionShape1, glm::quat rotationShape1, Shape* shape1, glm::vec3 positionShape2, glm::quat rotationShape2, Shape* shape2);
size_t CollideBoxCapsule(Contact* contacts, glm::vec3 positionShape1, glm::quat rotationShape1, Shape* shape1, glm::vec3 positionShape2, glm::quat rotationShape2, Shape* shape2);
size_t CollideSphereSphere(Contact* contacts, glm::vec3 positionShape1, glm::quat rotationShape1, Shape* shape1, glm::vec3 positionShape2, glm::quat rotationShape2, Shape* shape2);
size_t CollideSphereCapsule(Contact* contacts, glm::vec3 positionShape1, glm::quat rotationShape1, Shape* shape1, glm::vec3 positionShape2, glm::quat rotationShape2, Shape* shape2);
size_t CollideCapsuleCapsule(Contact* contacts, glm::vec3 positionShape1, glm::quat rotationShape1, Shape* shape1, glm::vec3 positionShape2, glm::quat rotationShape2, Shape* shape2);
size_t Collide(Contact* contacts, Body* body1, Shape* shape1, Body* body2, Shape* shape2);