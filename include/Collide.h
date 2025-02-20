#pragma once

#include <glm/glm.hpp>
#include <Body.h>

constexpr size_t g_maxContactPoints = 4;

struct Contact
{
    Contact()
    : m_separation(0.0f)
    , m_Pn(0.0f)
    , m_Pt(0.0f)
    , m_massNormal(0.0f)
    , m_massTangent(0.0f)
    , m_bias(0.0f)
    , m_feature(0)
    {
    }

    glm::vec3 m_position;
    glm::vec3 m_normal;
    glm::vec3 m_r1;
    glm::vec3 m_r2;
    float m_separation;
    float m_Pn;
    float m_Pt;
    float m_massNormal;
    float m_massTangent;
    float m_bias;
    uint32_t m_feature;
};

size_t CollideBoxBox(Contact* contacts, glm::vec3 positionShape1, glm::quat rotationShape1, Shape* shape1, glm::vec3 positionShape2, glm::quat rotationShape2, Shape* shape2);
size_t CollideBoxSphere(Contact* contacts, glm::vec3 positionShape1, glm::quat rotationShape1, Shape* shape1, glm::vec3 positionShape2, glm::quat rotationShape2, Shape* shape2);
size_t CollideBoxCapsule(Contact* contacts, glm::vec3 positionShape1, glm::quat rotationShape1, Shape* shape1, glm::vec3 positionShape2, glm::quat rotationShape2, Shape* shape2);
size_t CollideSphereSphere(Contact* contacts, glm::vec3 positionShape1, glm::quat rotationShape1, Shape* shape1, glm::vec3 positionShape2, glm::quat rotationShape2, Shape* shape2);
size_t CollideSphereCapsule(Contact* contacts, glm::vec3 positionShape1, glm::quat rotationShape1, Shape* shape1, glm::vec3 positionShape2, glm::quat rotationShape2, Shape* shape2);
size_t CollideCapsuleCapsule(Contact* contacts, glm::vec3 positionShape1, glm::quat rotationShape1, Shape* shape1, glm::vec3 positionShape2, glm::quat rotationShape2, Shape* shape2);
size_t Collide(Contact* contacts, Body* body1, Shape* shape1, Body* body2, Shape* shape2);