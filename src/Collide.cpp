#include "Collide.h"

struct OBB
{
    OBB(const glm::vec3& c, const glm::vec3& e, const glm::mat3& r)
    : m_center(c), m_halfExtents(e), m_rotation(r)
    {
    }

    glm::vec3 m_center;
    glm::vec3 m_halfExtents;
    glm::mat3 m_rotation;
};

struct CollisionInfo
{
    glm::vec3 m_position;
    glm::vec3 m_normal;
    float m_separation;
    uint32_t m_feature;
};

void ComputeOBBCorners(const OBB& obb, glm::vec3* corners)
{
    glm::vec3 rotatedX = obb.m_rotation * glm::vec3(obb.m_halfExtents.x, 0, 0);
    glm::vec3 rotatedY = obb.m_rotation * glm::vec3(0, obb.m_halfExtents.y, 0);
    glm::vec3 rotatedZ = obb.m_rotation * glm::vec3(0, 0, obb.m_halfExtents.z);
    corners[0] = obb.m_center - rotatedX - rotatedY - rotatedZ;
    corners[1] = obb.m_center + rotatedX - rotatedY - rotatedZ;
    corners[2] = obb.m_center - rotatedX + rotatedY - rotatedZ;
    corners[3] = obb.m_center + rotatedX + rotatedY - rotatedZ;
    corners[4] = obb.m_center - rotatedX - rotatedY + rotatedZ;
    corners[5] = obb.m_center + rotatedX - rotatedY + rotatedZ;
    corners[6] = obb.m_center - rotatedX + rotatedY + rotatedZ;
    corners[7] = obb.m_center + rotatedX + rotatedY + rotatedZ;
}

void ProjectOBB(const glm::vec3* corners, const glm::vec3& axis, float* minProj, float* maxProj)
{
    *minProj = std::numeric_limits<float>::infinity();
    *maxProj = -std::numeric_limits<float>::infinity();
    for (size_t i = 0; i < 8; ++i)
    {
        float projection = glm::dot(corners[i], axis);
        *minProj = std::min(*minProj, projection);
        *maxProj = std::max(*maxProj, projection);
    }
}

void ClipPointsPlane(const glm::vec3* points, const glm::vec3& planeNormal, float planeOffset, size_t* indices, size_t* indexCount)
{
    *indexCount = 0;
    for (size_t i = 0; i < 8; ++i)
    {
        float distanceToPlane = glm::dot(points[i], planeNormal) - planeOffset;
        if (distanceToPlane <= 0.0f)
        {
            indices[*indexCount] = i;
            ++(*indexCount);
        }
    }
}

void ComputeClosestPointsOnEdges(const glm::vec3& p1, const glm::vec3& q1, const glm::vec3& p2, const glm::vec3& q2, glm::vec3& c1, glm::vec3& c2)
{
    glm::vec3 d1 = q1 - p1;
    glm::vec3 d2 = q2 - p2;
    glm::vec3 r = p1 - p2;

    float a = glm::dot(d1, d1);
    float e = glm::dot(d2, d2);
    float f = glm::dot(d2, r);

    float s, t;
    if (a <= 1.0e-6f && e <= 1.0e-6f)
    {
        s = t = 0.0f;
        c1 = p1;
        c2 = p2;
        return;
    }
    if (a <= 1.0e-6f)
    {
        s = 0.0f;
        t = glm::clamp(f / e, 0.0f, 1.0f);
    }
    else
    {
        float c = glm::dot(d1, r);
        if (e <= 1.0e-6f)
        {
            t = 0.0f;
            s = glm::clamp(-c / a, 0.0f, 1.0f);
        }
        else
        {
            float b = glm::dot(d1, d2);
            float denom = a * e - b * b;

            if (denom != 0.0f)
            {
                s = glm::clamp((b * f - c * e) / denom, 0.0f, 1.0f);
            }
            else
            {
                s = 0.0f;
            }

            t = (b * s + f) / e;

            if (t < 0.0f)
            {
                t = 0.0f;
                s = glm::clamp(-c / a, 0.0f, 1.0f);
            }
            else if (t > 1.0f)
            {
                t = 1.0f;
                s = glm::clamp((b - c) / a, 0.0f, 1.0f);
            }
        }
    }

    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
}

void SeparatingAxisTheorem(const OBB& obb1, const OBB& obb2, size_t maxCollisionInfo, CollisionInfo* collisionInfos, size_t* count)
{
    *count = 0;

    size_t counter = 0;
    glm::vec3 axes[15];
    bool axeEnabled[15];
    for (size_t i = 0; i < 3; ++i)
    {
        axes[counter] = obb1.m_rotation[i];
        axeEnabled[counter] = true;
        ++counter;
    }
    for (size_t i = 0; i < 3; ++i)
    {
        axes[counter] = obb2.m_rotation[i];
        axeEnabled[counter] = true;
        ++counter;
    }
    for (size_t i = 0; i < 3; ++i)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            glm::vec3 crossAxis = glm::cross(obb1.m_rotation[i], obb2.m_rotation[j]);
            if (glm::length(crossAxis) > 1.0e-4f)
            {
                axes[counter] = glm::normalize(crossAxis);
                axeEnabled[counter] = true;
            }
            else
            {
                axeEnabled[counter] = false;
            }
            ++counter;
        }
    }

    glm::vec3 obb1Corners[8];
    ComputeOBBCorners(obb1, obb1Corners);

    glm::vec3 obb2Corners[8];
    ComputeOBBCorners(obb2, obb2Corners);

    float minSeparation = std::numeric_limits<float>::infinity();
    glm::vec3 collisionNormal;
    size_t bestAxisIndex;
    for (size_t i = 0; i < 15; ++i)
    {
        if (!axeEnabled[i])
        {
            continue;
        }

        const auto& axis = axes[i];

        float minProj1;
        float maxProj1;
        ProjectOBB(obb1Corners, axis, &minProj1, &maxProj1);

        float minProj2;
        float maxProj2;
        ProjectOBB(obb2Corners, axis, &minProj2, &maxProj2);

        float overlap = std::min(maxProj1, maxProj2) - std::max(minProj1, minProj2);
        if (overlap <= 0.0f)
        {
            return;
        }

        if (overlap < minSeparation)
        {
            minSeparation = overlap;
            collisionNormal = axis;
            bestAxisIndex = i;
        }
    }

    glm::vec3 T = obb2.m_center - obb1.m_center;
    if (glm::dot(collisionNormal, T) < 0.0f)
    {
        collisionNormal = -collisionNormal;
    }

    if (bestAxisIndex < 6)
    {
        // Face-face collision.
        const OBB& referenceBox = (bestAxisIndex < 3) ? obb1 : obb2;
        const OBB& incidentBox = (bestAxisIndex < 3) ? obb2 : obb1;
        const glm::vec3* incidentCorners = (bestAxisIndex < 3) ? obb2Corners : obb1Corners;

        size_t referenceAxisIndex = bestAxisIndex % 3;
        glm::vec3 referenceFaceNormal = referenceBox.m_rotation[referenceAxisIndex];
        if (bestAxisIndex >= 3)
        {
            referenceFaceNormal = -referenceFaceNormal;
        }

        glm::vec3 faceCenter = referenceBox.m_center + referenceFaceNormal * referenceBox.m_halfExtents[referenceAxisIndex];
        float planeOffset = glm::dot(faceCenter, referenceFaceNormal);

        size_t indices[8];
        size_t indexCount;
        ClipPointsPlane(incidentCorners, referenceFaceNormal, planeOffset, indices, &indexCount);

        const size_t clampedIndexCount = std::min(indexCount, maxCollisionInfo);
        for (size_t i = 0; i < clampedIndexCount; ++i)
        {
            CollisionInfo& collisionInfo = collisionInfos[*count];
            collisionInfo.m_position = incidentCorners[indices[i]];
            collisionInfo.m_normal = collisionNormal;
            collisionInfo.m_separation = minSeparation;
            collisionInfo.m_feature = referenceAxisIndex * 10 + indices[i];
            ++(*count);
        }
    }
    else
    {
        // Edge-edge collision.
        size_t edge1Index = (bestAxisIndex - 6) / 3;
        size_t edge2Index = (bestAxisIndex - 6) % 3;

        glm::vec3 edge1Dir = obb1.m_rotation[edge1Index] * obb1.m_halfExtents[edge1Index];
        glm::vec3 edge2Dir = obb2.m_rotation[edge2Index] * obb2.m_halfExtents[edge2Index];

        glm::vec3 edge1Start = obb1.m_center - edge1Dir;
        glm::vec3 edge1End = obb1.m_center + edge1Dir;

        glm::vec3 edge2Start = obb2.m_center - edge2Dir;
        glm::vec3 edge2End = obb2.m_center + edge2Dir;

        glm::vec3 closestPoint1;
        glm::vec3 closestPoint2;
        ComputeClosestPointsOnEdges(edge1Start, edge1End, edge2Start, edge2End, closestPoint1, closestPoint2);

        CollisionInfo& collisionInfo = collisionInfos[*count];
        collisionInfo.m_position = (closestPoint1 + closestPoint2) * 0.5f;
        collisionInfo.m_normal = collisionNormal;
        collisionInfo.m_separation = minSeparation;
        collisionInfo.m_feature = edge1Index * 10 + edge2Index * 100;
        ++(*count);
    }
}

size_t CollideBoxBox(Contact* contacts, glm::vec3 positionShape1, glm::quat rotationShape1, Shape* shape1, glm::vec3 positionShape2, glm::quat rotationShape2, Shape* shape2)
{
    ShapeBox* shapeBox1 = static_cast<ShapeBox*>(shape1);
    ShapeBox* shapeBox2 = static_cast<ShapeBox*>(shape2);

    OBB obb1 = OBB(positionShape1, shapeBox1->m_halfSize, glm::mat3_cast(rotationShape1));
    OBB obb2 = OBB(positionShape2, shapeBox2->m_halfSize, glm::mat3_cast(rotationShape2));

    CollisionInfo collisionInfos[g_maxContactPoints];
    size_t count;
    SeparatingAxisTheorem(obb1, obb2, g_maxContactPoints, collisionInfos, &count);

    for (size_t i = 0; i < count; ++i)
    {
        contacts[i].m_position = collisionInfos[i].m_position;
        contacts[i].m_normal = collisionInfos[i].m_normal;
        contacts[i].m_separation = collisionInfos[i].m_separation;
        contacts[i].m_feature = collisionInfos[i].m_feature;
    }

    return count;
}

size_t CollideBoxSphere(Contact* contacts, glm::vec3 positionShape1, glm::quat rotationShape1, Shape* shape1, glm::vec3 positionShape2, glm::quat rotationShape2, Shape* shape2)
{
    ShapeBox* shapeBox = static_cast<ShapeBox*>(shape1);
    ShapeSphere* shapeSphere = static_cast<ShapeSphere*>(shape2);
    // TODO
    return 0;
}

size_t CollideBoxCapsule(Contact* contacts, glm::vec3 positionShape1, glm::quat rotationShape1, Shape* shape1, glm::vec3 positionShape2, glm::quat rotationShape2, Shape* shape2)
{
    ShapeBox* shapeBox = static_cast<ShapeBox*>(shape1);
    ShapeCapsule* shapeCapsule = static_cast<ShapeCapsule*>(shape2);
    // TODO
    return 0;
}

size_t CollideSphereSphere(Contact* contacts, glm::vec3 positionShape1, glm::quat rotationShape1, Shape* shape1, glm::vec3 positionShape2, glm::quat rotationShape2, Shape* shape2)
{
    ShapeSphere* shapeSphere1 = static_cast<ShapeSphere*>(shape1);
    ShapeSphere* shapeSphere2 = static_cast<ShapeSphere*>(shape2);

    const glm::vec3 shape1ToShape2 = positionShape2 - positionShape1;
    const float shape1ToShape2Length = glm::length(shape1ToShape2);

    const float overlap = shape1ToShape2Length - shapeSphere1->m_radius - shapeSphere2->m_radius;
    if (overlap > 0.0f)
    {
        return 0;
    }

    contacts[0].m_normal = (shape1ToShape2Length > 0.0f) ? shape1ToShape2 * (1.0f / shape1ToShape2Length) : glm::vec3(0.0f, 1.0f, 0.0f);
    contacts[0].m_position = positionShape1 + contacts[0].m_normal * (shapeSphere1->m_radius + (overlap * 0.5f));
    contacts[0].m_separation = -overlap;
    contacts[0].m_feature = 0;

    return 1;
}

size_t CollideSphereCapsule(Contact* contacts, glm::vec3 positionShape1, glm::quat rotationShape1, Shape* shape1, glm::vec3 positionShape2, glm::quat rotationShape2, Shape* shape2)
{
    ShapeSphere* shapeSphere = static_cast<ShapeSphere*>(shape1);
    ShapeCapsule* shapeCapsule = static_cast<ShapeCapsule*>(shape2);
    // TODO
    return 0;
}

size_t CollideCapsuleCapsule(Contact* contacts, glm::vec3 positionShape1, glm::quat rotationShape1, Shape* shape1, glm::vec3 positionShape2, glm::quat rotationShape2, Shape* shape2)
{
    ShapeCapsule* shapeCapsule1 = static_cast<ShapeCapsule*>(shape1);
    ShapeCapsule* shapeCapsule2 = static_cast<ShapeCapsule*>(shape2);
    // TODO
    return 0;
}

size_t Collide(Contact* contacts, Body* body1, Shape* shape1, Body* body2, Shape* shape2)
{
    constexpr size_t shapeCount = static_cast<size_t>(ShapeType::Count);
    static const std::function<size_t(Contact*, glm::vec3, glm::quat, Shape*, glm::vec3, glm::quat, Shape*)> collisionMatrix[shapeCount][shapeCount]
    {
        {CollideBoxBox, CollideBoxSphere,    CollideBoxCapsule},
        {nullptr,       CollideSphereSphere, CollideSphereCapsule},
        {nullptr,       nullptr,             CollideCapsuleCapsule}
    };
    Body* lowestBody;
    Shape* lowestShape;
    Body* highestBody;
    Shape* highestShape;
    if (shape1->GetType() <= shape2->GetType())
    {
        lowestBody = body1;
        lowestShape = shape1;
        highestBody = body2;
        highestShape = shape2;
    }
    else
    {
        lowestBody = body2;
        lowestShape = shape2;
        highestBody = body1;
        highestShape = shape1;
    }
    const size_t shape1Type = static_cast<size_t>(lowestShape->GetType());
    const size_t shape2Type = static_cast<size_t>(highestShape->GetType());
    const glm::vec3 worldPositionShape1 = (lowestBody->m_rotation * lowestShape->m_position) + lowestBody->m_position;
    const glm::vec3 worldPositionShape2 = (highestBody->m_rotation * highestShape->m_position) + highestBody->m_position;
    const glm::quat worldRotationShape1 = lowestBody->m_rotation * lowestShape->m_rotation;
    const glm::quat worldRotationShape2 = highestBody->m_rotation * highestShape->m_rotation;
    return collisionMatrix[shape1Type][shape2Type](contacts, worldPositionShape1, worldRotationShape1, lowestShape, worldPositionShape2, worldRotationShape2, highestShape);
}