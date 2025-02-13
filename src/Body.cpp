#include "box2d-lite/Body.h"

GeometryBox::GeometryBox()
{
    halfSize = glm::vec3(0.5f, 0.5f, 0.5f);
}

void GeometryBox::Set(const glm::vec3& hs, float m)
{
    halfSize = hs;
}

glm::vec3 GeometryBox::ComputeI(float m) const
{
    if ((m > 0.0f) && (m < FLT_MAX))
    {
        glm::vec3 s = 2.0f * halfSize;
        constexpr float f = 1.0f / 12.0f;
        return glm::vec3(
            m * (s.y * s.y + s.z * s.z) * f,
            m * (s.x * s.x + s.z * s.z) * f,
            m * (s.x * s.x + s.y * s.y) * f);
    }
    else
    {
        return glm::vec3(0.0f, 0.0f, 0.0f);
    }
}

GeometrySphere::GeometrySphere()
{
    radius = 0.5f;
}

void GeometrySphere::Set(float r, float m)
{
    radius = r;
}

glm::vec3 GeometrySphere::ComputeI(float m) const
{
    if ((m > 0.0f) && (m < FLT_MAX))
    {
        float v = 1.0f / ((2.0f / 5.0f) * m * radius * radius);
        return glm::vec3(v, v, v);
    }
    else
    {
        return glm::vec3(0.0f, 0.0f, 0.0f);
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
}

glm::vec3 GeometryCapsule::ComputeI(float m) const
{
    if ((m > 0.0f) && (m < FLT_MAX))
    {
        float volumeCylinder = glm::pi<float>() * radius * radius * (2.0f * halfHeight);
        float volumeHemispheres = (4.0f / 3.0f) * glm::pi<float>() * radius * radius * radius;
        float totalVolume = volumeCylinder + volumeHemispheres;

        float massCylinder = m * (volumeCylinder / totalVolume);
        float massHemisphere = 0.5f * (m - massCylinder);

        float IcTransverse = (1.0f / 12.0f) * massCylinder * (3.0f * radius * radius + 4.0f * halfHeight * halfHeight);
        float IcLongitudinal = 0.5f * massCylinder * radius * radius;

        float Ih = (2.0f / 5.0f) * massHemisphere * radius * radius;
        float IhOffset = massHemisphere * (halfHeight + (3.0f / 8.0f) * radius) * (halfHeight + (3.0f / 8.0f) * radius);

        float Ixz = IcTransverse + 2.0f * (Ih + IhOffset);
        float Iy = IcLongitudinal + 2.0f * Ih;
        return glm::vec3(Ixz, Iy, Ixz);
    }
    else
    {
        return glm::vec3(0.0f, 0.0f, 0.0f);
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
    invI = glm::mat3(0.0f);
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
    shape.geometry.Set(s, m);
    if ((mass > 0.0f) && (mass < FLT_MAX))
    {
        invMass = 1.0f / mass;
        glm::vec3 I = shape.geometry.ComputeI(m);
        invI = glm::mat3(
            glm::vec3((I.x > 0.0f) ? 1.0f / I.x : 0.0f, 0.0f, 0.0f),
            glm::vec3(0.0f, (I.y > 0.0f) ? 1.0f / I.y : 0.0f, 0.0f),
            glm::vec3(0.0f, 0.0f, (I.z > 0.0f) ? 1.0f / I.z : 0.0f));
    }
    else
    {
        invMass = 0.0f;
        invI = glm::mat3(0.0f);
    }
}

void Body::AddForce(const glm::vec3& f)
{
    force += f;
}