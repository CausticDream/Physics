#include "box2d-lite/Arbiter.h"
#include "box2d-lite/Body.h"
#include <array>

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

void GetOBBCorners(const OBB& obb, std::array<glm::vec3, 8>& corners)
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

std::pair<float, float> ProjectOBB(const std::array<glm::vec3, 8>& corners, const glm::vec3& axis)
{
    float minProj = std::numeric_limits<float>::infinity();
    float maxProj = -std::numeric_limits<float>::infinity();
    for (const auto& corner : corners)
    {
        float projection = glm::dot(corner, axis);
        minProj = std::min(minProj, projection);
        maxProj = std::max(maxProj, projection);
    }
    return { minProj, maxProj };
}

float IntervalOverlap(const std::pair<float, float>& interval1, const std::pair<float, float>& interval2)
{
    return std::max(0.0f, std::min(interval1.second, interval2.second) - std::max(interval1.first, interval2.first));
}

std::vector<int> ClipPointsPlane(const std::array<glm::vec3, 8>& points, const glm::vec3& planeNormal, float planeOffset)
{
    std::vector<int> clippedPoints;
    for (int i = 0; i < points.size(); ++i)
    {
        float distanceToPlane = glm::dot(points[i], planeNormal) - planeOffset;
        if (distanceToPlane <= 0.0f)
        {
            clippedPoints.push_back(i);
        }
    }
    return clippedPoints;
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

std::vector<CollisionInfo> SeparatingAxisTheorem(const OBB& obb1, const OBB& obb2)
{
    std::vector<glm::vec3> axes;
    for (int i = 0; i < 3; ++i)
    {
        axes.push_back(obb1.rotation[i]);
    }
    for (int i = 0; i < 3; ++i)
    {
        axes.push_back(obb2.rotation[i]);
    }
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            glm::vec3 crossAxis = glm::cross(obb1.rotation[i], obb2.rotation[j]);
            if (glm::length(crossAxis) > 1.0e-4f)
            {
                axes.push_back(glm::normalize(crossAxis));
            }
        }
    }

    std::array<glm::vec3, 8> obb1Corners;
    GetOBBCorners(obb1, obb1Corners);

    std::array<glm::vec3, 8> obb2Corners;
    GetOBBCorners(obb2, obb2Corners);

    float minSeparation = std::numeric_limits<float>::infinity();
    glm::vec3 collisionNormal;
    int bestAxisIndex = -1;
    for (size_t i = 0; i < axes.size(); ++i)
    {
        const auto& axis = axes[i];

        auto proj1 = ProjectOBB(obb1Corners, axis);
        auto proj2 = ProjectOBB(obb2Corners, axis);
        float overlap = IntervalOverlap(proj1, proj2);
        if (overlap <= 0.0f)
        {
            return {};
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

    std::vector<CollisionInfo> collisions;
    if (bestAxisIndex < 6)
    {
        // Face-face collision.
        const OBB& referenceBox = (bestAxisIndex < 3) ? obb1 : obb2;
        const OBB& incidentBox = (bestAxisIndex < 3) ? obb2 : obb1;
        const std::array<glm::vec3, 8>& incidentCorners = (bestAxisIndex < 3) ? obb2Corners : obb1Corners;

        int referenceAxisIndex = bestAxisIndex % 3;
        glm::vec3 referenceFaceNormal = referenceBox.rotation[referenceAxisIndex];
        if (bestAxisIndex >= 3)
        {
            referenceFaceNormal = -referenceFaceNormal;
        }

        glm::vec3 faceCenter = referenceBox.center + referenceFaceNormal * referenceBox.halfExtents[referenceAxisIndex];
        float planeOffset = glm::dot(faceCenter, referenceFaceNormal);

        std::vector<int> contactPoints = ClipPointsPlane(incidentCorners, referenceFaceNormal, planeOffset);
        const size_t pointsToAdd = std::min<size_t>(contactPoints.size(), 4);
        for (size_t i = 0; i < pointsToAdd; ++i)
        {
            CollisionInfo collisionInfo;
            collisionInfo.position = incidentCorners[contactPoints[i]];
            collisionInfo.normal = collisionNormal;
            collisionInfo.separation = minSeparation;
            collisionInfo.feature = static_cast<int>(referenceAxisIndex * 10 + contactPoints[i]);
            collisions.push_back(collisionInfo);
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

        glm::vec3 closestPoint1, closestPoint2;
        ComputeClosestPointsOnEdges(edge1Start, edge1End, edge2Start, edge2End, closestPoint1, closestPoint2);

        CollisionInfo collisionInfo1;
        collisionInfo1.position = closestPoint1;
        collisionInfo1.normal = collisionNormal;
        collisionInfo1.separation = minSeparation;
        collisionInfo1.feature = static_cast<int>(edge1Index * 10 + edge2Index * 100);
        collisions.push_back(collisionInfo1);

        CollisionInfo collisionInfo2;
        collisionInfo2.position = closestPoint2;
        collisionInfo2.normal = collisionNormal;
        collisionInfo2.separation = minSeparation;
        collisionInfo2.feature = static_cast<int>(edge1Index * 10 + edge2Index * 100 + 1);
        collisions.push_back(collisionInfo2);
    }
    return collisions;
}

int CollideBoxBox(Contact* contacts, Body* body1, ShapeBox* shape1, Body* body2, ShapeBox* shape2)
{
    OBB obb1 = OBB(body1->position, shape1->halfSize, glm::mat3_cast(body1->rotation));
    OBB obb2 = OBB(body2->position, shape2->halfSize, glm::mat3_cast(body2->rotation));
    std::vector<CollisionInfo> collisionInfo = SeparatingAxisTheorem(obb1, obb2);
    for (size_t i = 0; i < collisionInfo.size(); ++i)
    {
        contacts[i].position = collisionInfo[i].position;
        contacts[i].normal = collisionInfo[i].normal;
        contacts[i].separation = collisionInfo[i].separation;
        contacts[i].feature = collisionInfo[i].feature;
    }
    return collisionInfo.size();
}