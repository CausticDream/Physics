#include "Collide.h"

struct OBB
{
    OBB(const glm::vec3& c, const glm::vec3& e, const glm::mat3& r)
    : center(c), halfExtents(e), rotation(r)
    {
    }

    glm::vec3 center;
    glm::vec3 halfExtents;
    glm::mat3 rotation;
};

struct CollisionInfo
{
    glm::vec3 position;
    glm::vec3 normal;
    float separation;
    int feature;
};

void ComputeOBBCorners(const OBB& obb, glm::vec3* corners)
{
    glm::vec3 rotatedX = obb.rotation * glm::vec3(obb.halfExtents.x, 0, 0);
    glm::vec3 rotatedY = obb.rotation * glm::vec3(0, obb.halfExtents.y, 0);
    glm::vec3 rotatedZ = obb.rotation * glm::vec3(0, 0, obb.halfExtents.z);
    corners[0] = obb.center - rotatedX - rotatedY - rotatedZ;
    corners[1] = obb.center + rotatedX - rotatedY - rotatedZ;
    corners[2] = obb.center - rotatedX + rotatedY - rotatedZ;
    corners[3] = obb.center + rotatedX + rotatedY - rotatedZ;
    corners[4] = obb.center - rotatedX - rotatedY + rotatedZ;
    corners[5] = obb.center + rotatedX - rotatedY + rotatedZ;
    corners[6] = obb.center - rotatedX + rotatedY + rotatedZ;
    corners[7] = obb.center + rotatedX + rotatedY + rotatedZ;
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

void ClipPointsPlane(const glm::vec3* points, const glm::vec3& planeNormal, float planeOffset, int* indices, size_t* indexCount)
{
    *indexCount = 0;
    for (int i = 0; i < 8; ++i)
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
    for (int i = 0; i < 3; ++i)
    {
        axes[counter] = obb1.rotation[i];
        axeEnabled[counter] = true;
        ++counter;
    }
    for (int i = 0; i < 3; ++i)
    {
        axes[counter] = obb2.rotation[i];
        axeEnabled[counter] = true;
        ++counter;
    }
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            glm::vec3 crossAxis = glm::cross(obb1.rotation[i], obb2.rotation[j]);
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
    int bestAxisIndex = -1;
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
            bestAxisIndex = static_cast<int>(i);
        }
    }

    glm::vec3 T = obb2.center - obb1.center;
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

        int referenceAxisIndex = bestAxisIndex % 3;
        glm::vec3 referenceFaceNormal = referenceBox.rotation[referenceAxisIndex];
        if (bestAxisIndex >= 3)
        {
            referenceFaceNormal = -referenceFaceNormal;
        }

        glm::vec3 faceCenter = referenceBox.center + referenceFaceNormal * referenceBox.halfExtents[referenceAxisIndex];
        float planeOffset = glm::dot(faceCenter, referenceFaceNormal);

        int indices[8];
        size_t indexCount;
        ClipPointsPlane(incidentCorners, referenceFaceNormal, planeOffset, indices, &indexCount);

        const size_t clampedIndexCount = std::min(indexCount, maxCollisionInfo);
        for (size_t i = 0; i < clampedIndexCount; ++i)
        {
            CollisionInfo& collisionInfo = collisionInfos[*count];
            collisionInfo.position = incidentCorners[indices[i]];
            collisionInfo.normal = collisionNormal;
            collisionInfo.separation = minSeparation;
            collisionInfo.feature = static_cast<int>(referenceAxisIndex * 10 + indices[i]);
            ++(*count);
        }
    }
    else
    {
        // Edge-edge collision.
        int edge1Index = (bestAxisIndex - 6) / 3;
        int edge2Index = (bestAxisIndex - 6) % 3;

        glm::vec3 edge1Dir = obb1.rotation[edge1Index] * obb1.halfExtents[edge1Index];
        glm::vec3 edge2Dir = obb2.rotation[edge2Index] * obb2.halfExtents[edge2Index];

        glm::vec3 edge1Start = obb1.center - edge1Dir;
        glm::vec3 edge1End = obb1.center + edge1Dir;

        glm::vec3 edge2Start = obb2.center - edge2Dir;
        glm::vec3 edge2End = obb2.center + edge2Dir;

        glm::vec3 closestPoint1;
        glm::vec3 closestPoint2;
        ComputeClosestPointsOnEdges(edge1Start, edge1End, edge2Start, edge2End, closestPoint1, closestPoint2);

        CollisionInfo& collisionInfo = collisionInfos[*count];
        collisionInfo.position = (closestPoint1 + closestPoint2) * 0.5f;
        collisionInfo.normal = collisionNormal;
        collisionInfo.separation = minSeparation;
        collisionInfo.feature = static_cast<int>(edge1Index * 10 + edge2Index * 100);
        ++(*count);
    }
}

size_t CollideBoxBox(Contact* contacts, glm::vec3 positionShape1, glm::quat rotationShape1, Shape* shape1, glm::vec3 positionShape2, glm::quat rotationShape2, Shape* shape2)
{
    ShapeBox* shapeBox1 = static_cast<ShapeBox*>(shape1);
    ShapeBox* shapeBox2 = static_cast<ShapeBox*>(shape2);

    OBB obb1 = OBB(positionShape1, shapeBox1->halfSize, glm::mat3_cast(rotationShape1));
    OBB obb2 = OBB(positionShape2, shapeBox2->halfSize, glm::mat3_cast(rotationShape2));

    CollisionInfo collisionInfos[MAX_CONTACT_POINTS];
    size_t count;
    SeparatingAxisTheorem(obb1, obb2, MAX_CONTACT_POINTS, collisionInfos, &count);

    for (size_t i = 0; i < count; ++i)
    {
        contacts[i].position = collisionInfos[i].position;
        contacts[i].normal = collisionInfos[i].normal;
        contacts[i].separation = collisionInfos[i].separation;
        contacts[i].feature = collisionInfos[i].feature;
    }

    return count;
}

size_t CollideBoxSphere(Contact* contacts, glm::vec3 positionShape1, glm::quat rotationShape1, Shape* shape1, glm::vec3 positionShape2, glm::quat rotationShape2, Shape* shape2)
{
    // TODO
    return 0;
}

size_t CollideBoxCapsule(Contact* contacts, glm::vec3 positionShape1, glm::quat rotationShape1, Shape* shape1, glm::vec3 positionShape2, glm::quat rotationShape2, Shape* shape2)
{
    // TODO
    return 0;
}

size_t CollideSphereSphere(Contact* contacts, glm::vec3 positionShape1, glm::quat rotationShape1, Shape* shape1, glm::vec3 positionShape2, glm::quat rotationShape2, Shape* shape2)
{
    ShapeSphere* shapeSphere1 = static_cast<ShapeSphere*>(shape1);
    ShapeSphere* shapeSphere2 = static_cast<ShapeSphere*>(shape2);

    const float overlap = glm::length(positionShape2 - positionShape1) - shapeSphere1->radius - shapeSphere2->radius;
    if (overlap > 0.0f)
    {
        return 0;
    }

    contacts[0].normal = glm::normalize(positionShape2 - positionShape1);
    contacts[0].position = positionShape1 + contacts[0].normal * (shapeSphere1->radius + (overlap * 0.5f));
    contacts[0].separation = -overlap;
    contacts[0].feature = 0; // TODO

    return 1;
}

size_t CollideSphereCapsule(Contact* contacts, glm::vec3 positionShape1, glm::quat rotationShape1, Shape* shape1, glm::vec3 positionShape2, glm::quat rotationShape2, Shape* shape2)
{
    // TODO
    return 0;
}

size_t CollideCapsuleCapsule(Contact* contacts, glm::vec3 positionShape1, glm::quat rotationShape1, Shape* shape1, glm::vec3 positionShape2, glm::quat rotationShape2, Shape* shape2)
{
    // TODO
    return 0;
}

size_t Collide(Contact* contacts, Body* body1, Shape* shape1, Body* body2, Shape* shape2)
{
    constexpr size_t shapeCount = static_cast<size_t>(ShapeType::Count);
    static const std::function<size_t(Contact*, glm::vec3, glm::quat, Shape*, glm::vec3, glm::quat, Shape*)> collisionMatrix[shapeCount][shapeCount]
    {
        {CollideBoxBox,     CollideBoxSphere,     CollideBoxCapsule},
        {CollideBoxSphere,  CollideSphereSphere,  CollideSphereCapsule},
        {CollideBoxCapsule, CollideSphereCapsule, CollideCapsuleCapsule}
    };
    const glm::vec3 worldPositionShape1 = (body1->rotation * shape1->position) + body1->position;
    const glm::vec3 worldPositionShape2 = (body2->rotation * shape2->position) + body2->position;
    const glm::quat worldRotationShape1 = body1->rotation * shape1->rotation;
    const glm::quat worldRotationShape2 = body2->rotation * shape2->rotation;
    const size_t shape1Type = static_cast<size_t>(shape1->GetType());
    const size_t shape2Type = static_cast<size_t>(shape2->GetType());
    return collisionMatrix[shape1Type][shape2Type](contacts, worldPositionShape1, worldRotationShape1, shape1, worldPositionShape2, worldRotationShape2, shape2);
}