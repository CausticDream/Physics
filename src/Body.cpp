#include "Body.h"
#include <atomic>

std::atomic<uint32_t> g_counter;

Material::Material()
: staticFriction(0.5f)
, dynamicFriction(0.5f)
, restitution(0.0f)
, frictionCombineMode(CombineMode::Average)
, restitutionCombineMode(CombineMode::Average)
{
}

Shape::~Shape()
{
    if (owner)
    {
        owner->shapes.erase(std::find(owner->shapes.begin(), owner->shapes.end(), this));
    }
}

Shape::Shape(ShapeType t)
{
    uniqueID = g_counter++;
    owner = nullptr;
    position = glm::vec3(0.0f, 0.0f, 0.0f);
    rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    type = t;
}

ShapeBox::ShapeBox()
: Shape(ShapeType::Box)
{
    halfSize = glm::vec3(0.5f, 0.5f, 0.5f);
}

void ShapeBox::Set(const glm::vec3& hs)
{
    halfSize = hs;

    if (owner)
    {
        owner->ComputeInvI();
    }
}

ShapeSphere::ShapeSphere()
: Shape(ShapeType::Sphere)
{
    radius = 0.5f;
}

void ShapeSphere::Set(float r)
{
    radius = r;

    if (owner)
    {
        owner->ComputeInvI();
    }
}

ShapeCapsule::ShapeCapsule()
: Shape(ShapeType::Capsule)
{
    radius = 0.5f;
    halfHeight = 0.5f;
}

void ShapeCapsule::Set(float r, float hh)
{
    radius = r;
    halfHeight = hh;

    if (owner)
    {
        owner->ComputeInvI();
    }
}

Body::Body()
{
    position = glm::vec3(0.0f, 0.0f, 0.0f);
    rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    velocity = glm::vec3(0.0f, 0.0f, 0.0f);
    angularVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
    force = glm::vec3(0.0f, 0.0f, 0.0f);
    torque = glm::vec3(0.0f, 0.0f, 0.0f);
    mass = std::numeric_limits<float>::infinity();
    invMass = 0.0f;
    invI = glm::mat3(0.0f);
}

Body::~Body()
{
    while (!shapes.empty())
    {
        delete shapes[0];
    }
}

void Body::SetMass(float m)
{
    mass = m;
    if ((mass > 0.0f) && (mass < std::numeric_limits<float>::infinity()))
    {
        invMass = 1.0f / mass;
    }
    else
    {
        invMass = 0.0f;
    }
    ComputeInvI();
}

void Body::AddForce(const glm::vec3& f)
{
    force += f;
}

void Body::AddShape(Shape* shape)
{
    shape->owner = this;
    shapes.push_back(shape);
}

void Body::ComputeInvI()
{
    if ((mass > 0.0f) && (mass < std::numeric_limits<float>::infinity()))
    {
        glm::vec3 I = glm::vec3(0.0f, 0.0f, 0.0f);
        for (size_t i = 0; i < shapes.size(); ++i)
        {
            const Shape* s = shapes[i];

            switch (s->GetType())
            {
                case ShapeType::Box:
                {
                    const ShapeBox* shapeBox = static_cast<const ShapeBox*>(s);
                    glm::vec3 s = 2.0f * shapeBox->halfSize;
                    constexpr float f = 1.0f / 12.0f;
                    I += glm::vec3(
                        mass * (s.y * s.y + s.z * s.z) * f,
                        mass * (s.x * s.x + s.z * s.z) * f,
                        mass * (s.x * s.x + s.y * s.y) * f);
                    break;
                }

                case ShapeType::Sphere:
                {
                    const ShapeSphere* shapeSphere = static_cast<const ShapeSphere*>(s);
                    float v = 1.0f / ((2.0f / 5.0f) * mass * shapeSphere->radius * shapeSphere->radius);
                    I += glm::vec3(v, v, v);
                    break;
                }

                case ShapeType::Capsule:
                {
                    const ShapeCapsule* shapeCapsule = static_cast<const ShapeCapsule*>(s);

                    float volumeCylinder = glm::pi<float>() * shapeCapsule->radius * shapeCapsule->radius * (2.0f * shapeCapsule->halfHeight);
                    float volumeHemispheres = (4.0f / 3.0f) * glm::pi<float>() * shapeCapsule->radius * shapeCapsule->radius * shapeCapsule->radius;
                    float totalVolume = volumeCylinder + volumeHemispheres;

                    float massCylinder = mass * (volumeCylinder / totalVolume);
                    float massHemisphere = 0.5f * (mass - massCylinder);

                    float IcTransverse = (1.0f / 12.0f) * massCylinder * (3.0f * shapeCapsule->radius * shapeCapsule->radius + 4.0f * shapeCapsule->halfHeight * shapeCapsule->halfHeight);
                    float IcLongitudinal = 0.5f * massCylinder * shapeCapsule->radius * shapeCapsule->radius;

                    float Ih = (2.0f / 5.0f) * massHemisphere * shapeCapsule->radius * shapeCapsule->radius;
                    float IhOffset = massHemisphere * (shapeCapsule->halfHeight + (3.0f / 8.0f) * shapeCapsule->radius) * (shapeCapsule->halfHeight + (3.0f / 8.0f) * shapeCapsule->radius);

                    float Ixz = IcTransverse + 2.0f * (Ih + IhOffset);
                    float Iy = IcLongitudinal + 2.0f * Ih;
                    I += glm::vec3(Ixz, Iy, Ixz);
                    break;
                }
            }
        }
        invI = glm::mat3(
            glm::vec3((I.x > 0.0f) ? 1.0f / I.x : 0.0f, 0.0f, 0.0f),
            glm::vec3(0.0f, (I.y > 0.0f) ? 1.0f / I.y : 0.0f, 0.0f),
            glm::vec3(0.0f, 0.0f, (I.z > 0.0f) ? 1.0f / I.z : 0.0f));
    }
    else
    {
        invI = glm::mat3(0.0f);
    }
}