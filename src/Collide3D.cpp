#include "box2d-lite/Arbiter3D.h"
#include "box2d-lite/Body3D.h"

enum Axis
{
    FACE_A_X,
    FACE_A_Y,
    FACE_A_Z,
    FACE_B_X,
    FACE_B_Y,
    FACE_B_Z
};

enum FaceNumbers
{
    NO_FACE = 0,
    FACE1,
    FACE2,
    FACE3,
    FACE4,
    FACE5,
    FACE6
};

struct ClipVertex
{
    ClipVertex()
    {
        fp.value = 0;
    }
    glm::vec3 v; // Vertex position
    Feature3DPair fp; // Feature information
};

void Flip(Feature3DPair& fp)
{
    std::swap(fp.f.inFace1, fp.f.inFace2);
    std::swap(fp.f.outFace1, fp.f.outFace2);
}

int ClipPolygonToPlane(ClipVertex vOut[], ClipVertex vIn[], int numIn, const glm::vec3& normal, float offset, char clipFace)
{
    int numOut = 0;
    ClipVertex prevVertex = vIn[numIn - 1];
    float prevDistance = glm::dot(normal, prevVertex.v) - offset;

    for (int i = 0; i < numIn; ++i)
    {
        ClipVertex currVertex = vIn[i];
        float currDistance = glm::dot(normal, currVertex.v) - offset;

        if (currDistance <= 0.0f)
        {
            if (prevDistance > 0.0f)
            {
                // Add intersection point
                float t = prevDistance / (prevDistance - currDistance);
                vOut[numOut].v =
                    prevVertex.v + t * (currVertex.v - prevVertex.v);
                vOut[numOut].fp = prevVertex.fp;
                vOut[numOut].fp.f.inFace1 = clipFace;
                ++numOut;
            }
            vOut[numOut++] = currVertex;
        }
        else if (prevDistance <= 0.0f)
        {
            // Add intersection point
            float t = prevDistance / (prevDistance - currDistance);
            vOut[numOut].v =
                prevVertex.v + t * (currVertex.v - prevVertex.v);
            vOut[numOut].fp = currVertex.fp;
            vOut[numOut].fp.f.outFace1 = clipFace;
            ++numOut;
        }

        prevVertex = currVertex;
        prevDistance = currDistance;
    }

    return numOut;
}

static void ComputeIncidentFace(ClipVertex c[4], const glm::vec3& h, const glm::vec3& pos, const glm::mat3& Rot, const glm::vec3& normal)
{
    glm::mat3 RotT = glm::transpose(Rot);
    glm::vec3 n = -(RotT * normal); // Transform normal to local space of the box
    glm::vec3 nAbs = glm::abs(n);

    if (nAbs.x > nAbs.y && nAbs.x > nAbs.z)
    {
        if (n.x > 0.0f)
        {
            c[0].v = pos + Rot * glm::vec3(h.x, -h.y, -h.z);
            c[1].v = pos + Rot * glm::vec3(h.x, h.y, -h.z);
            c[2].v = pos + Rot * glm::vec3(h.x, h.y, h.z);
            c[3].v = pos + Rot * glm::vec3(h.x, -h.y, h.z);

            c[0].fp.f.inFace2 = FACE5;
            c[1].fp.f.inFace2 = FACE5;
            c[2].fp.f.inFace2 = FACE5;
            c[3].fp.f.inFace2 = FACE5;
        }
        else
        {
            c[0].v = pos + Rot * glm::vec3(-h.x, h.y, h.z);
            c[1].v = pos + Rot * glm::vec3(-h.x, h.y, -h.z);
            c[2].v = pos + Rot * glm::vec3(-h.x, -h.y, -h.z);
            c[3].v = pos + Rot * glm::vec3(-h.x, -h.y, h.z);

            c[0].fp.f.inFace2 = FACE6;
            c[1].fp.f.inFace2 = FACE6;
            c[2].fp.f.inFace2 = FACE6;
            c[3].fp.f.inFace2 = FACE6;
        }
    }
    else if (nAbs.y > nAbs.z)
    {
        if (n.y > 0.0f)
        {
            c[0].v = pos + Rot * glm::vec3(-h.x, h.y, -h.z);
            c[1].v = pos + Rot * glm::vec3(h.x, h.y, -h.z);
            c[2].v = pos + Rot * glm::vec3(h.x, h.y, h.z);
            c[3].v = pos + Rot * glm::vec3(-h.x, h.y, h.z);

            c[0].fp.f.inFace2 = FACE4;
            c[1].fp.f.inFace2 = FACE4;
            c[2].fp.f.inFace2 = FACE4;
            c[3].fp.f.inFace2 = FACE4;
        }
        else
        {
            c[0].v = pos + Rot * glm::vec3(h.x, -h.y, h.z);
            c[1].v = pos + Rot * glm::vec3(h.x, -h.y, -h.z);
            c[2].v = pos + Rot * glm::vec3(-h.x, -h.y, -h.z);
            c[3].v = pos + Rot * glm::vec3(-h.x, -h.y, h.z);

            c[0].fp.f.inFace2 = FACE3;
            c[1].fp.f.inFace2 = FACE3;
            c[2].fp.f.inFace2 = FACE3;
            c[3].fp.f.inFace2 = FACE3;
        }
    }
    else
    {
        if (n.z > 0.0f)
        {
            c[0].v = pos + Rot * glm::vec3(-h.x, -h.y, h.z);
            c[1].v = pos + Rot * glm::vec3(h.x, -h.y, h.z);
            c[2].v = pos + Rot * glm::vec3(h.x, h.y, h.z);
            c[3].v = pos + Rot * glm::vec3(-h.x, h.y, h.z);

            c[0].fp.f.inFace2 = FACE1;
            c[1].fp.f.inFace2 = FACE1;
            c[2].fp.f.inFace2 = FACE1;
            c[3].fp.f.inFace2 = FACE1;
        }
        else
        {
            c[0].v = pos + Rot * glm::vec3(h.x, h.y, -h.z);
            c[1].v = pos + Rot * glm::vec3(-h.x, h.y, -h.z);
            c[2].v = pos + Rot * glm::vec3(-h.x, -h.y, -h.z);
            c[3].v = pos + Rot * glm::vec3(h.x, -h.y, -h.z);

            c[0].fp.f.inFace2 = FACE2;
            c[1].fp.f.inFace2 = FACE2;
            c[2].fp.f.inFace2 = FACE2;
            c[3].fp.f.inFace2 = FACE2;
        }
    }
}

int Collide3D(Contact3D* contacts, Body3D* body1, Body3D* body2)
{
    // Setup
    glm::vec3 hA = 0.5f * body1->width; // Half extents of box A
    glm::vec3 hB = 0.5f * body2->width; // Half extents of box B

    glm::vec3 posA = body1->position;
    glm::vec3 posB = body2->position;

    glm::mat3 rotA = glm::mat3_cast(body1->rotation); // Convert quaternion to rotation matrix
    glm::mat3 rotB = glm::mat3_cast(body2->rotation);

    glm::mat3 rotAT = glm::transpose(rotA);
    glm::mat3 rotBT = glm::transpose(rotB);

    glm::vec3 dp = posB - posA; // Vector between box centers in world space
    glm::vec3 dA = rotAT * dp; // dp in box A's local space
    glm::vec3 dB = rotBT * dp; // dp in box B's local space

    glm::mat3 C = rotAT * rotB; // Rotation from B's local space to A's local space
    glm::mat3 absC(
        glm::vec3(std::abs(C[0][0]), std::abs(C[0][1]), std::abs(C[0][2])),
        glm::vec3(std::abs(C[1][0]), std::abs(C[1][1]), std::abs(C[1][2])),
        glm::vec3(std::abs(C[2][0]), std::abs(C[2][1]), std::abs(C[2][2])));
    glm::mat3 absCT = glm::transpose(absC);

    // Check faces of box A
    glm::vec3 faceA = glm::abs(dA) - hA - absC * hB;
    if (faceA.x > 0.0f || faceA.y > 0.0f || faceA.z > 0.0f)
    {
        return 0;
    }

    // Check faces of box B
    glm::vec3 faceB = glm::abs(dB) - absCT * hA - hB;
    if (faceB.x > 0.0f || faceB.y > 0.0f || faceB.z > 0.0f)
    {
        return 0;
    }

    // Find the best axis of separation
    Axis axis;
    float separation;
    glm::vec3 normal;

    // Box A faces
    axis = FACE_A_X;
    separation = faceA.x;
    normal = dA.x > 0.0f ? rotA[0] : -rotA[0];

    if (faceA.y > separation)
    {
        axis = FACE_A_Y;
        separation = faceA.y;
        normal = dA.y > 0.0f ? rotA[1] : -rotA[1];
    }

    if (faceA.z > separation)
    {
        axis = FACE_A_Z;
        separation = faceA.z;
        normal = dA.z > 0.0f ? rotA[2] : -rotA[2];
    }

    // Box B faces
    if (faceB.x > separation)
    {
        axis = FACE_B_X;
        separation = faceB.x;
        normal = dB.x > 0.0f ? rotB[0] : -rotB[0];
    }

    if (faceB.y > separation)
    {
        axis = FACE_B_Y;
        separation = faceB.y;
        normal = dB.y > 0.0f ? rotB[1] : -rotB[1];
    }

    if (faceB.z > separation)
    {
        axis = FACE_B_Z;
        separation = faceB.z;
        normal = dB.z > 0.0f ? rotB[2] : -rotB[2];
    }

    const float relativeTol = 0.95f; // Relative tolerance for SAT tests
    const float absoluteTol = 0.01f; // Absolute tolerance for SAT tests

    // Check edge cross products (12 axes in total for edge-edge tests)
    for (int i = 0; i < 3; ++i) // Axes of box A
    {
        for (int j = 0; j < 3; ++j) // Axes of box B
        {
            glm::vec3 axisTest = glm::cross(rotA[i], rotB[j]);
            if (glm::length(axisTest) < absoluteTol)
            {
                continue; // Skip near-zero axes
            }

            axisTest = glm::normalize(axisTest);
            float projectionA =
                hA.x * std::abs(glm::dot(axisTest, rotA[0])) +
                hA.y * std::abs(glm::dot(axisTest, rotA[1])) +
                hA.z * std::abs(glm::dot(axisTest, rotA[2]));

            float projectionB =
                hB.x * std::abs(glm::dot(axisTest, rotB[0])) +
                hB.y * std::abs(glm::dot(axisTest, rotB[1])) +
                hB.z * std::abs(glm::dot(axisTest, rotB[2]));

            float distance =
                std::abs(glm::dot(dp, axisTest)) - projectionA - projectionB;

            if (distance > relativeTol * separation + absoluteTol)
            {
                return 0;
            }

            if (distance > separation)
            {
                separation = distance;
                normal = axisTest;
            }
        }
    }

    // Setup clipping plane data based on the separating axis
    ClipVertex incidentFace[4];
    ClipVertex clipPoints1[4], clipPoints2[4];

    float front, negSide, posSide;
    char negFace, posFace;

    glm::vec3 frontNormal, sideNormal;

    // Compute the clipping lines and the line segment to be clipped.
    switch (axis)
    {
        case FACE_A_X:
        {
            frontNormal = normal;
            front = glm::dot(posA, frontNormal) + hA.x;
            sideNormal = rotA[1];
            float side = glm::dot(posA, sideNormal);
            negSide = -side + hA.y;
            posSide = side + hA.y;
            negFace = FACE3;
            posFace = FACE4;
            ComputeIncidentFace(incidentFace, hB, posB, rotB, frontNormal);
        }
        break;

        case FACE_A_Y:
        {
            frontNormal = normal;
            front = glm::dot(posA, frontNormal) + hA.y;
            sideNormal = rotA[0];
            float side = glm::dot(posA, sideNormal);
            negSide = -side + hA.x;
            posSide = side + hA.x;
            negFace = FACE5;
            posFace = FACE6;
            ComputeIncidentFace(incidentFace, hB, posB, rotB, frontNormal);
        }
        break;

        case FACE_A_Z:
        {
            frontNormal = normal;
            front = glm::dot(posA, frontNormal) + hA.z;
            sideNormal = rotA[0];
            float side = glm::dot(posA, sideNormal);
            negSide = -side + hA.x;
            posSide = side + hA.x;
            negFace = FACE1;
            posFace = FACE2;
            ComputeIncidentFace(incidentFace, hB, posB, rotB, frontNormal);
        }
        break;

        case FACE_B_X:
        {
            frontNormal = -normal;
            front = glm::dot(posB, frontNormal) + hB.x;
            sideNormal = rotB[1];
            float side = glm::dot(posB, sideNormal);
            negSide = -side + hB.y;
            posSide = side + hB.y;
            negFace = FACE3;
            posFace = FACE4;
            ComputeIncidentFace(incidentFace, hA, posA, rotA, frontNormal);
        }
        break;

        case FACE_B_Y:
        {
            frontNormal = -normal;
            front = glm::dot(posB, frontNormal) + hB.y;
            sideNormal = rotB[0];
            float side = glm::dot(posB, sideNormal);
            negSide = -side + hB.x;
            posSide = side + hB.x;
            negFace = FACE5;
            posFace = FACE6;
            ComputeIncidentFace(incidentFace, hA, posA, rotA, frontNormal);
        }
        break;

        case FACE_B_Z:
        {
            frontNormal = -normal;
            front = glm::dot(posB, frontNormal) + hB.z;
            sideNormal = rotB[0];
            float side = glm::dot(posB, sideNormal);
            negSide = -side + hB.x;
            posSide = side + hB.x;
            negFace = FACE1;
            posFace = FACE2;
            ComputeIncidentFace(incidentFace, hA, posA, rotA, frontNormal);
        }
        break;

        default:
            return 0; // Invalid axis
    }

    // Clip against the planes of the reference face
    int numContacts;

    numContacts =
        ClipPolygonToPlane(clipPoints1,
                           incidentFace,
                           4,
                           -sideNormal,
                           negSide,
                           negFace);

    if (numContacts < 2)
    {
        return 0;
    }

    numContacts =
        ClipPolygonToPlane(clipPoints2,
                           clipPoints1,
                           numContacts,
                           sideNormal,
                           posSide,
                           posFace);

    if (numContacts < 2)
    {
        return 0;
    }

    // Now clipPoints2 contains the clipping points.
    // Due to roundoff errors or degeneracies in geometry,
    // it is possible that clipping removes all points.

    int contactCount = 0;

    for (int i = 0; i < numContacts; ++i)
    {
        float separation =
            glm::dot(frontNormal,
                     clipPoints2[i].v) -
            front;

        if (separation <= 0.0f)
        {
            // Add contact point
            contacts[contactCount].separation = separation;
            contacts[contactCount].normal = normal;
            contacts[contactCount].position = clipPoints2[i].v - separation * frontNormal; // Project point onto reference face
            contacts[contactCount].feature = clipPoints2[i].fp;

            // Flip the feature pair if the axis is from box B
            if (axis == FACE_B_X || axis == FACE_B_Y || axis == FACE_B_Z)
            {
                Flip(contacts[contactCount].feature);
            }

            ++contactCount;
        }
    }

    return contactCount;
}