#include "Body.h"
#include <atomic>

std::atomic<uint32_t> g_counter;

Material::Material()
: m_staticFriction(0.5f)
, m_dynamicFriction(0.5f)
, m_restitution(0.0f)
, m_frictionCombineMode(CombineMode::Average)
, m_restitutionCombineMode(CombineMode::Average)
{
}

Shape::~Shape()
{
    if (m_owner)
    {
        m_owner->m_shapes.erase(std::find(m_owner->m_shapes.begin(), m_owner->m_shapes.end(), this));
    }
}

Shape::Shape(ShapeType t)
{
    m_uniqueID = g_counter++;
    m_owner = nullptr;
    m_position = glm::vec3(0.0f, 0.0f, 0.0f);
    m_rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    m_type = t;
}

ShapeBox::ShapeBox()
: Shape(ShapeType::Box)
{
    m_halfSize = glm::vec3(0.5f, 0.5f, 0.5f);
}

void ShapeBox::Set(const glm::vec3& halfSize)
{
    m_halfSize = halfSize;

    if (m_owner)
    {
        m_owner->ComputeInvI();
    }
}

glm::vec3 ShapeBox::ComputeI(float mass) const
{
    glm::vec3 s = 2.0f * m_halfSize;
    constexpr float f = 1.0f / 12.0f;
    return glm::vec3(
        mass * (s.y * s.y + s.z * s.z) * f,
        mass * (s.x * s.x + s.z * s.z) * f,
        mass * (s.x * s.x + s.y * s.y) * f);
}

ShapeSphere::ShapeSphere()
: Shape(ShapeType::Sphere)
{
    m_radius = 0.5f;
}

void ShapeSphere::Set(float radius)
{
    m_radius = radius;

    if (m_owner)
    {
        m_owner->ComputeInvI();
    }
}

glm::vec3 ShapeSphere::ComputeI(float mass) const
{
    float v = 1.0f / ((2.0f / 5.0f) * mass * m_radius * m_radius);
    return glm::vec3(v, v, v);
}

ShapeCapsule::ShapeCapsule()
: Shape(ShapeType::Capsule)
{
    m_radius = 0.5f;
    m_halfHeight = 0.5f;
}

void ShapeCapsule::Set(float radius, float halfHeight)
{
    m_radius = radius;
    m_halfHeight = halfHeight;

    if (m_owner)
    {
        m_owner->ComputeInvI();
    }
}

glm::vec3 ShapeCapsule::ComputeI(float mass) const
{
    float volumeCylinder = glm::pi<float>() * m_radius * m_radius * (2.0f * m_halfHeight);
    float volumeHemispheres = (4.0f / 3.0f) * glm::pi<float>() * m_radius * m_radius * m_radius;
    float totalVolume = volumeCylinder + volumeHemispheres;

    float massCylinder = mass * (volumeCylinder / totalVolume);
    float massHemisphere = 0.5f * (mass - massCylinder);

    float IcTransverse = (1.0f / 12.0f) * massCylinder * (3.0f * m_radius * m_radius + 4.0f * m_halfHeight * m_halfHeight);
    float IcLongitudinal = 0.5f * massCylinder * m_radius * m_radius;

    float Ih = (2.0f / 5.0f) * massHemisphere * m_radius * m_radius;
    float IhOffset = massHemisphere * (m_halfHeight + (3.0f / 8.0f) * m_radius) * (m_halfHeight + (3.0f / 8.0f) * m_radius);

    float Ixz = IcTransverse + 2.0f * (Ih + IhOffset);
    float Iy = IcLongitudinal + 2.0f * Ih;
    return glm::vec3(Ixz, Iy, Ixz);
}

Body::Body()
{
    userData = nullptr;
    m_position = glm::vec3(0.0f, 0.0f, 0.0f);
    m_rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    m_velocity = glm::vec3(0.0f, 0.0f, 0.0f);
    m_angularVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
    m_force = glm::vec3(0.0f, 0.0f, 0.0f);
    m_torque = glm::vec3(0.0f, 0.0f, 0.0f);
    m_mass = std::numeric_limits<float>::infinity();
    m_invMass = 0.0f;
    m_invI = glm::mat3(0.0f);
}

Body::~Body()
{
    while (!m_shapes.empty())
    {
        delete m_shapes[0];
    }
}

void Body::SetMass(float mass)
{
    m_mass = mass;
    if ((m_mass > 0.0f) && (m_mass < std::numeric_limits<float>::infinity()))
    {
        m_invMass = 1.0f / m_mass;
    }
    else
    {
        m_invMass = 0.0f;
    }
    ComputeInvI();
}

void Body::AddForce(const glm::vec3& force)
{
    m_force += force;
}

void Body::AddShape(Shape* shape)
{
    shape->m_owner = this;
    m_shapes.push_back(shape);
}

void Body::ComputeInvI()
{
    if ((m_mass > 0.0f) && (m_mass < std::numeric_limits<float>::infinity()))
    {
        glm::vec3 I = glm::vec3(0.0f, 0.0f, 0.0f);
        for (size_t i = 0; i < m_shapes.size(); ++i)
        {
            const Shape* s = m_shapes[i];

            switch (s->GetType())
            {
                case ShapeType::Box:
                {
                    const ShapeBox* shapeBox = static_cast<const ShapeBox*>(s);
                    I += shapeBox->ComputeI(m_mass);
                    break;
                }

                case ShapeType::Sphere:
                {
                    const ShapeSphere* shapeSphere = static_cast<const ShapeSphere*>(s);
                    I += shapeSphere->ComputeI(m_mass);
                    break;
                }

                case ShapeType::Capsule:
                {
                    const ShapeCapsule* shapeCapsule = static_cast<const ShapeCapsule*>(s);
                    I += shapeCapsule->ComputeI(m_mass);
                    break;
                }
            }
        }
        m_invI = glm::mat3(
            glm::vec3((I.x > 0.0f) ? 1.0f / I.x : 0.0f, 0.0f, 0.0f),
            glm::vec3(0.0f, (I.y > 0.0f) ? 1.0f / I.y : 0.0f, 0.0f),
            glm::vec3(0.0f, 0.0f, (I.z > 0.0f) ? 1.0f / I.z : 0.0f));
    }
    else
    {
        m_invI = glm::mat3(0.0f);
    }
}