#include "box2d-lite/Arbiter.h"
#include "box2d-lite/Body.h"

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
    ClipVertex() { fp.value = 0; }
    glm::vec3 v;
    FeaturePair fp;
};

void Flip(FeaturePair& fp)
{
    std::swap(fp.f.inFace1, fp.f.inFace2);
    std::swap(fp.f.outFace1, fp.f.outFace2);
}

int ClipSegmentToLine(ClipVertex vOut[2], ClipVertex vIn[2], const glm::vec3& normal, float offset, char clipFace)
{
    int numOut = 0;
    float distance0 = glm::dot(normal, vIn[0].v) - offset;
    float distance1 = glm::dot(normal, vIn[1].v) - offset;

    if (distance0 <= 0.0f) vOut[numOut++] = vIn[0];
    if (distance1 <= 0.0f) vOut[numOut++] = vIn[1];

    if (distance0 * distance1 < 0.0f)
    {
        float interp = distance0 / (distance0 - distance1);
        vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
        if (distance0 > 0.0f)
        {
            vOut[numOut].fp = vIn[0].fp;
            vOut[numOut].fp.f.inFace1 = clipFace;
            vOut[numOut].fp.f.inFace2 = NO_FACE;
        }
        else
        {
            vOut[numOut].fp = vIn[1].fp;
            vOut[numOut].fp.f.outFace1 = clipFace;
            vOut[numOut].fp.f.outFace2 = NO_FACE;
        }
        ++numOut;
    }

    return numOut;
}

static void ComputeIncidentFace(ClipVertex c[4], const glm::vec3& h, const glm::vec3& pos, const glm::quat& rot, const glm::vec3& normal)
{
    glm::mat3 rotMat = glm::mat3_cast(rot);
    glm::vec3 n = -glm::inverse(rotMat) * normal;
    glm::vec3 nAbs = glm::abs(n);

    if (nAbs.x > nAbs.y && nAbs.x > nAbs.z)
    {
        if (n.x > 0.0f)
        {
            c[0].v = glm::vec3(h.x, -h.y, -h.z);
            c[1].v = glm::vec3(h.x, -h.y, h.z);
            c[2].v = glm::vec3(h.x, h.y, h.z);
            c[3].v = glm::vec3(h.x, h.y, -h.z);
            c[0].fp.f.inFace2 = FACE1;
            c[1].fp.f.inFace2 = FACE1;
            c[2].fp.f.inFace2 = FACE1;
            c[3].fp.f.inFace2 = FACE1;
        }
        else
        {
            c[0].v = glm::vec3(-h.x, -h.y, h.z);
            c[1].v = glm::vec3(-h.x, -h.y, -h.z);
            c[2].v = glm::vec3(-h.x, h.y, -h.z);
            c[3].v = glm::vec3(-h.x, h.y, h.z);
            c[0].fp.f.inFace2 = FACE2;
            c[1].fp.f.inFace2 = FACE2;
            c[2].fp.f.inFace2 = FACE2;
            c[3].fp.f.inFace2 = FACE2;
        }
    }
    else if (nAbs.y > nAbs.z)
    {
        if (n.y > 0.0f)
        {
            c[0].v = glm::vec3(-h.x, h.y, -h.z);
            c[1].v = glm::vec3(h.x, h.y, -h.z);
            c[2].v = glm::vec3(h.x, h.y, h.z);
            c[3].v = glm::vec3(-h.x, h.y, h.z);
            c[0].fp.f.inFace2 = FACE3;
            c[1].fp.f.inFace2 = FACE3;
            c[2].fp.f.inFace2 = FACE3;
            c[3].fp.f.inFace2 = FACE3;
        }
        else
        {
            c[0].v = glm::vec3(-h.x, -h.y, h.z);
            c[1].v = glm::vec3(h.x, -h.y, h.z);
            c[2].v = glm::vec3(h.x, -h.y, -h.z);
            c[3].v = glm::vec3(-h.x, -h.y, -h.z);
            c[0].fp.f.inFace2 = FACE4;
            c[1].fp.f.inFace2 = FACE4;
            c[2].fp.f.inFace2 = FACE4;
            c[3].fp.f.inFace2 = FACE4;
        }
    }
    else
    {
        if (n.z > 0.0f)
        {
            c[0].v = glm::vec3(-h.x, -h.y, h.z);
            c[1].v = glm::vec3(h.x, -h.y, h.z);
            c[2].v = glm::vec3(h.x, h.y, h.z);
            c[3].v = glm::vec3(-h.x, h.y, h.z);
            c[0].fp.f.inFace2 = FACE5;
            c[1].fp.f.inFace2 = FACE5;
            c[2].fp.f.inFace2 = FACE5;
            c[3].fp.f.inFace2 = FACE5;
        }
        else
        {
            c[0].v = glm::vec3(h.x, -h.y, -h.z);
            c[1].v = glm::vec3(-h.x, -h.y, -h.z);
            c[2].v = glm::vec3(-h.x, h.y, -h.z);
            c[3].v = glm::vec3(h.x, h.y, -h.z);
            c[0].fp.f.inFace2 = FACE6;
            c[1].fp.f.inFace2 = FACE6;
            c[2].fp.f.inFace2 = FACE6;
            c[3].fp.f.inFace2 = FACE6;
        }
    }

    for (int i = 0; i < 4; ++i)
        c[i].v = pos + glm::mat3_cast(rot) * c[i].v;
}

int Collide(Contact* contacts, Body* bodyA, Body* bodyB)
{
    glm::vec3 hA = 0.5f * bodyA->size;
    glm::vec3 hB = 0.5f * bodyB->size;
    glm::vec3 posA = bodyA->position;
    glm::vec3 posB = bodyB->position;
    glm::mat3 RotA = glm::mat3_cast(bodyA->rotation);
    glm::mat3 RotB = glm::mat3_cast(bodyB->rotation);
    glm::mat3 RotAT = glm::transpose(RotA);
    glm::mat3 RotBT = glm::transpose(RotB);
    glm::vec3 dp = posB - posA;
    glm::vec3 dA = RotAT * dp;
    glm::vec3 dB = RotBT * dp;
    glm::mat3 C = RotAT * RotB;
    glm::mat3 absC(
        glm::vec3(std::abs(C[0][0]), std::abs(C[0][1]), std::abs(C[0][2])),
        glm::vec3(std::abs(C[1][0]), std::abs(C[1][1]), std::abs(C[1][2])),
        glm::vec3(std::abs(C[2][0]), std::abs(C[2][1]), std::abs(C[2][2])));
    glm::mat3 absCT = glm::transpose(absC);

    glm::vec3 faceA = glm::abs(dA) - hA - absC * hB;
    if (faceA.x > 0.0f || faceA.y > 0.0f || faceA.z > 0.0f)
        return 0;

    glm::vec3 faceB = glm::abs(dB) - absCT * hA - hB;
    if (faceB.x > 0.0f || faceB.y > 0.0f || faceB.z > 0.0f)
        return 0;

    Axis axis;
    float separation;
    glm::vec3 normal;

    axis = FACE_A_X;
    separation = faceA.x;
    normal = dA.x > 0.0f ? RotA[0] : -RotA[0];

    const float relativeTol = 0.95f;
    const float absoluteTol = 0.01f;

    if (faceA.y > relativeTol * separation + absoluteTol * hA.y)
    {
        axis = FACE_A_Y;
        separation = faceA.y;
        normal = dA.y > 0.0f ? RotA[1] : -RotA[1];
    }

    if (faceA.z > relativeTol * separation + absoluteTol * hA.z)
    {
        axis = FACE_A_Z;
        separation = faceA.z;
        normal = dA.z > 0.0f ? RotA[2] : -RotA[2];
    }

    if (faceB.x > relativeTol * separation + absoluteTol * hB.x)
    {
        axis = FACE_B_X;
        separation = faceB.x;
        normal = dB.x > 0.0f ? RotB[0] : -RotB[0];
    }

    if (faceB.y > relativeTol * separation + absoluteTol * hB.y)
    {
        axis = FACE_B_Y;
        separation = faceB.y;
        normal = dB.y > 0.0f ? RotB[1] : -RotB[1];
    }

    if (faceB.z > relativeTol * separation + absoluteTol * hB.z)
    {
        axis = FACE_B_Z;
        separation = faceB.z;
        normal = dB.z > 0.0f ? RotB[2] : -RotB[2];
    }

    glm::vec3 frontNormal, sideNormal1, sideNormal2;
    ClipVertex incidentFace[4];
    float front, side1, side2;
    char sideFace1, sideFace2;

    switch (axis)
    {
    case FACE_A_X:
    {
        frontNormal = normal;
        front = glm::dot(posA, frontNormal) + hA.x;
        sideNormal1 = RotA[1];
        sideNormal2 = RotA[2];
        side1 = glm::dot(posA, sideNormal1);
        side2 = glm::dot(posA, sideNormal2);
        sideFace1 = FACE3;
        sideFace2 = FACE5;
        ComputeIncidentFace(incidentFace, hB, posB, bodyB->rotation, frontNormal);
        break;
    }
    case FACE_A_Y:
    {
        frontNormal = normal;
        front = glm::dot(posA, frontNormal) + hA.y;
        sideNormal1 = RotA[0];
        sideNormal2 = RotA[2];
        side1 = glm::dot(posA, sideNormal1);
        side2 = glm::dot(posA, sideNormal2);
        sideFace1 = FACE1;
        sideFace2 = FACE5;
        ComputeIncidentFace(incidentFace, hB, posB, bodyB->rotation, frontNormal);
        break;
    }
    case FACE_A_Z:
    {
        frontNormal = normal;
        front = glm::dot(posA, frontNormal) + hA.z;
        sideNormal1 = RotA[0];
        sideNormal2 = RotA[1];
        side1 = glm::dot(posA, sideNormal1);
        side2 = glm::dot(posA, sideNormal2);
        sideFace1 = FACE1;
        sideFace2 = FACE3;
        ComputeIncidentFace(incidentFace, hB, posB, bodyB->rotation, frontNormal);
        break;
    }
    case FACE_B_X:
    {
        frontNormal = -normal;
        front = glm::dot(posB, frontNormal) + hB.x;
        sideNormal1 = RotB[1];
        sideNormal2 = RotB[2];
        side1 = glm::dot(posB, sideNormal1);
        side2 = glm::dot(posB, sideNormal2);
        sideFace1 = FACE3;
        sideFace2 = FACE5;
        ComputeIncidentFace(incidentFace, hA, posA, bodyA->rotation, frontNormal);
        break;
    }
    case FACE_B_Y:
    {
        frontNormal = -normal;
        front = glm::dot(posB, frontNormal) + hB.y;
        sideNormal1 = RotB[0];
        sideNormal2 = RotB[2];
        side1 = glm::dot(posB, sideNormal1);
        side2 = glm::dot(posB, sideNormal2);
        sideFace1 = FACE1;
        sideFace2 = FACE5;
        ComputeIncidentFace(incidentFace, hA, posA, bodyA->rotation, frontNormal);
        break;
    }
    case FACE_B_Z:
    {
        frontNormal = -normal;
        front = glm::dot(posB, frontNormal) + hB.z;
        sideNormal1 = RotB[0];
        sideNormal2 = RotB[1];
        side1 = glm::dot(posB, sideNormal1);
        side2 = glm::dot(posB, sideNormal2);
        sideFace1 = FACE1;
        sideFace2 = FACE3;
        ComputeIncidentFace(incidentFace, hA, posA, bodyA->rotation, frontNormal);
        break;
    }
    }

    // clip other face with 5 box planes (1 face plane, 4 edge planes)

    ClipVertex clipPoints1[4];
    ClipVertex clipPoints2[4];
    int np;

    // Clip to box side 1
    np = ClipSegmentToLine(&clipPoints1[0], &incidentFace[0], -sideNormal1, -side1 + hA.y, sideFace1);
    np = ClipSegmentToLine(&clipPoints1[np], &incidentFace[2], -sideNormal1, -side1 + hA.y, sideFace1);
    if (np < 2)
        return 0;

    // Clip to negative box side 1
    np = ClipSegmentToLine(&clipPoints2[0], &clipPoints1[0], sideNormal1, side1 + hA.y, sideFace1);
    np = ClipSegmentToLine(&clipPoints2[np], &clipPoints1[2], sideNormal1, side1 + hA.y, sideFace1);
    if (np < 2)
        return 0;

    // Clip to box side 2
    np = ClipSegmentToLine(&clipPoints1[0], &clipPoints2[0], -sideNormal2, -side2 + hA.z, sideFace2);
    np = ClipSegmentToLine(&clipPoints1[np], &clipPoints2[2], -sideNormal2, -side2 + hA.z, sideFace2);

    if (np < 2)
        return 0;

    // Clip to negative box side 2
    np = ClipSegmentToLine(&clipPoints2[0], &clipPoints1[0], sideNormal2, side2 + hA.z, sideFace2);
    np = ClipSegmentToLine(&clipPoints2[np], &clipPoints1[2], sideNormal2, side2 + hA.z, sideFace2);
    if (np < 2)
        return 0;

    // Now clipPoints2 contains the clipping points.
    // Due to roundoff, it is possible that clipping removes all points.

    int numContacts = 0;
    for (int i = 0; i < 4; ++i)
    {
        float separation = glm::dot(frontNormal, clipPoints2[i].v) - front;
        if (separation <= 0)
        {
            contacts[numContacts].separation = separation;
            contacts[numContacts].normal = normal;
            // slide contact point onto reference face (easy to cull)
            contacts[numContacts].position = clipPoints2[i].v - separation * frontNormal;
            contacts[numContacts].feature = clipPoints2[i].fp;
            if (axis == FACE_B_X || axis == FACE_B_Y || axis == FACE_B_Z)
            {
                Flip(contacts[numContacts].feature);
            }

            ++numContacts;
        }
    }

    return numContacts;
}