#include "box2d-lite/Body.h"

Geometry::Geometry()
{
    invI = glm::mat3(0.0f);
}

GeometryBox::GeometryBox()
{
    halfSize = glm::vec3(0.5f, 0.5f, 0.5f);
}

void GeometryBox::Set(const glm::vec3& hs, float m)
{
    halfSize = hs;

    if ((m > 0.0f) && (m < FLT_MAX))
    {
        glm::vec3 s = 2.0f * hs;
        constexpr float f = 1.0f / 12.0f;
        glm::vec3 I = glm::vec3(
            m * (s.y * s.y + s.z * s.z) * f,
            m * (s.x * s.x + s.z * s.z) * f,
            m * (s.x * s.x + s.y * s.y) * f);

        invI = glm::mat3(
            glm::vec3(1.0f / I.x, 0.0f, 0.0f),
            glm::vec3(0.0f, 1.0f / I.y, 0.0f),
            glm::vec3(0.0f, 0.0f, 1.0f / I.z));
    }
    else
    {
        invI = glm::mat3(0.0f);
    }
}

GeometrySphere::GeometrySphere()
{
    radius = 0.5f;
}

void GeometrySphere::Set(float r, float m)
{
    radius = r;

    if ((m > 0.0f) && (m < FLT_MAX))
    {
        float v = 1.0f / ((2.0f / 5.0f) * m * r * r);
        invI = glm::mat3(
            glm::vec3(v, 0.0f, 0.0f),
            glm::vec3(0.0f, v, 0.0f),
            glm::vec3(0.0f, 0.0f, v));
    }
    else
    {
        invI = glm::mat3(0.0f);
    }
}

GeometryCapsule::GeometryCapsule()
{
    radius = 0.5f;
    halfHeight = 0.5f;
}

void GeometryCapsule::Set(float r, float hh, float m)
{
    radius = r;
    halfHeight = hh;

    if ((m > 0.0f) && (m < FLT_MAX))
    {
        float volumeCylinder = glm::pi<float>() * r * r * (2.0f * hh);
        float volumeSphere = (4.0f / 3.0f) * glm::pi<float>() * r * r * r;
        float totalVolume = volumeCylinder + volumeSphere;
        float massCylinder = m * (volumeCylinder / totalVolume);
        float massSphere = m - massCylinder;
        float Iy = (1.0f / 12.0f) * massCylinder * (3.0f * r * r + 4.0f * hh * hh) + (2.0f / 5.0f) * massSphere * r * r;
        float Ixz = (1.0f / 4.0f) * massCylinder * r * r + (1.0f / 12.0f) * massCylinder * (4.0f * hh * hh) + (2.0f / 5.0f) * massSphere * r * r + massSphere * hh * hh;
        invI = glm::mat3(
            glm::vec3(1.0f / Ixz, 0.0f, 0.0f),
            glm::vec3(0.0f, 1.0f / Iy, 0.0f),
            glm::vec3(0.0f, 0.0f, 1.0f / Ixz));
    }
    else
    {
        invI = glm::mat3(0.0f);
    }
}

Material::Material()
: staticFriction(0.5f)
, dynamicFriction(0.5f)
, restitution(0.0f)
, frictionCombineMode(CombineMode::Average)
, restitutionCombineMode(CombineMode::Average)
{
}

Body::Body()
{
    position = glm::vec3(0.0f, 0.0f, 0.0f);
    rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    velocity = glm::vec3(0.0f, 0.0f, 0.0f);
    angularVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
    force = glm::vec3(0.0f, 0.0f, 0.0f);
    torque = glm::vec3(0.0f, 0.0f, 0.0f);
    mass = FLT_MAX;
    invMass = 0.0f;
}

void Body::Set(const glm::vec3& s, float m)
{
    position = glm::vec3(0.0f, 0.0f, 0.0f);
    rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    velocity = glm::vec3(0.0f, 0.0f, 0.0f);
    angularVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
    force = glm::vec3(0.0f, 0.0f, 0.0f);
    torque = glm::vec3(0.0f, 0.0f, 0.0f);
    mass = m;
    if ((mass > 0.0f) && (mass < FLT_MAX))
    {
        invMass = 1.0f / mass;
    }
    else
    {
        invMass = 0.0f;
    }
    shape.geometry.Set(s, m);
}

void Body::AddForce(const glm::vec3& f)
{
    force += f;
}