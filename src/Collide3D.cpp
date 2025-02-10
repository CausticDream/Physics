#include "box2d-lite/Arbiter3D.h"
#include "box2d-lite/Body3D.h"

enum Axis3D
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

struct ClipVertex3D
{
    ClipVertex3D()
    {
        fp.value = 0;
    }

    glm::vec3 v;
    Feature3DPair fp;
};

void Flip3D(Feature3DPair& fp)
{
    std::swap(fp.f.inFace1, fp.f.inFace2);
    std::swap(fp.f.outFace1, fp.f.outFace2);
}

int ClipSegmentToPlane(ClipVertex3D vOut[2], ClipVertex3D vIn[2],
    const glm::vec3& normal, float offset, char clipFace)
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

static void ComputeIncidentFace(ClipVertex3D c[4], const glm::vec3& h, const glm::vec3& pos,
    const glm::mat3& Rot, const glm::vec3& normal)
{
    glm::mat3 RotT = glm::transpose(Rot);
    glm::vec3 n = -(RotT * normal);
    glm::vec3 nAbs = glm::abs(n);

    if (nAbs.x > nAbs.y && nAbs.x > nAbs.z)
    {
        if (n.x > 0.0f)
        {
            c[0].v = glm::vec3(h.x, -h.y, -h.z);
            c[1].v = glm::vec3(h.x, -h.y, h.z);
            c[2].v = glm::vec3(h.x, h.y, h.z);
            c[3].v = glm::vec3(h.x, h.y, -h.z);

            c[0].fp.f.inFace2 = FACE3;
            c[1].fp.f.inFace2 = FACE4;
            c[2].fp.f.inFace2 = FACE5;
            c[3].fp.f.inFace2 = FACE6;
        }
        else
        {
            c[0].v = glm::vec3(-h.x, -h.y, h.z);
            c[1].v = glm::vec3(-h.x, -h.y, -h.z);
            c[2].v = glm::vec3(-h.x, h.y, -h.z);
            c[3].v = glm::vec3(-h.x, h.y, h.z);

            c[0].fp.f.inFace2 = FACE1;
            c[1].fp.f.inFace2 = FACE2;
            c[2].fp.f.inFace2 = FACE6;
            c[3].fp.f.inFace2 = FACE5;
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

            c[0].fp.f.inFace2 = FACE2;
            c[1].fp.f.inFace2 = FACE6;
            c[2].fp.f.inFace2 = FACE5;
            c[3].fp.f.inFace2 = FACE1;
        }
        else
        {
            c[0].v = glm::vec3(-h.x, -h.y, h.z);
            c[1].v = glm::vec3(h.x, -h.y, h.z);
            c[2].v = glm::vec3(h.x, -h.y, -h.z);
            c[3].v = glm::vec3(-h.x, -h.y, -h.z);

            c[0].fp.f.inFace2 = FACE1;
            c[1].fp.f.inFace2 = FACE5;
            c[2].fp.f.inFace2 = FACE6;
            c[3].fp.f.inFace2 = FACE2;
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

            c[0].fp.f.inFace2 = FACE1;
            c[1].fp.f.inFace2 = FACE4;
            c[2].fp.f.inFace2 = FACE5;
            c[3].fp.f.inFace2 = FACE2;
        }
        else
        {
            c[0].v = glm::vec3(h.x, -h.y, -h.z);
            c[1].v = glm::vec3(-h.x, -h.y, -h.z);
            c[2].v = glm::vec3(-h.x, h.y, -h.z);
            c[3].v = glm::vec3(h.x, h.y, -h.z);

            c[0].fp.f.inFace2 = FACE4;
            c[1].fp.f.inFace2 = FACE3;
            c[2].fp.f.inFace2 = FACE2;
            c[3].fp.f.inFace2 = FACE6;
        }
    }

    for (int i = 0; i < 4; ++i)
    {
        c[i].v = pos + Rot * c[i].v;
    }
}

int Collide3D(Contact3D* contacts, Body3D* body1, Body3D* body2)
{
    glm::vec3 h1 = 0.5f * body1->width;
    glm::vec3 h2 = 0.5f * body2->width;
    glm::vec3 pos1 = body1->position;
    glm::vec3 pos2 = body2->position;
    glm::mat3 Rot1 = glm::mat3_cast(body1->rotation);
    glm::mat3 Rot2 = glm::mat3_cast(body2->rotation);
    glm::mat3 Rot1T = glm::transpose(Rot1);
    glm::mat3 Rot2T = glm::transpose(Rot2);
    glm::vec3 dp = pos2 - pos1;
    glm::vec3 d1 = Rot1T * dp;
    glm::vec3 d2 = Rot2T * dp;
    glm::mat3 C = Rot1T * Rot2;
    glm::mat3 absC(
        glm::vec3(std::abs(C[0][0]), std::abs(C[0][1]), std::abs(C[0][2])),
        glm::vec3(std::abs(C[1][0]), std::abs(C[1][1]), std::abs(C[1][2])),
        glm::vec3(std::abs(C[2][0]), std::abs(C[2][1]), std::abs(C[2][2])));
    glm::mat3 absCT = glm::transpose(absC);

    glm::vec3 face1 = glm::abs(d1) - h1 - absC * h2;
    if (face1.x > 0.0f || face1.y > 0.0f || face1.z > 0.0f)
        return 0;

    glm::vec3 face2 = glm::abs(d2) - absCT * h1 - h2;
    if (face2.x > 0.0f || face2.y > 0.0f || face2.z > 0.0f)
        return 0;

    Axis3D axis;
    float separation;
    glm::vec3 normal;

    axis = FACE_A_X;
    separation = face1.x;
    normal = d1.x > 0.0f ? Rot1[0] : -Rot1[0];

    const float relativeTol = 0.95f;
    const float absoluteTol = 0.01f;

    if (face1.y > relativeTol * separation + absoluteTol * h1.y)
    {
        axis = FACE_A_Y;
        separation = face1.y;
        normal = d1.y > 0.0f ? Rot1[1] : -Rot1[1];
    }

    if (face1.z > relativeTol * separation + absoluteTol * h1.z)
    {
        axis = FACE_A_Z;
        separation = face1.z;
        normal = d1.z > 0.0f ? Rot1[2] : -Rot1[2];
    }

    if (face2.x > relativeTol * separation + absoluteTol * h2.x)
    {
        axis = FACE_B_X;
        separation = face2.x;
        normal = d2.x > 0.0f ? Rot2[0] : -Rot2[0];
    }

    if (face2.y > relativeTol * separation + absoluteTol * h2.y)
    {
        axis = FACE_B_Y;
        separation = face2.y;
        normal = d2.y > 0.0f ? Rot2[1] : -Rot2[1];
    }

    if (face2.z > relativeTol * separation + absoluteTol * h2.z)
    {
        axis = FACE_B_Z;
        separation = face2.z;
        normal = d2.z > 0.0f ? Rot2[2] : -Rot2[2];
    }

    glm::vec3 frontNormal, sideNormal1, sideNormal2;
    ClipVertex3D incidentFace[4];
    float front, negSide1, posSide1, negSide2, posSide2;
    char negEdge1, posEdge1, negEdge2, posEdge2;

    switch (axis)
    {
    case FACE_A_X:
    {
        frontNormal = normal;
        front = glm::dot(pos1, frontNormal) + h1.x;
        sideNormal1 = Rot1[1];
        sideNormal2 = Rot1[2];
        float side1 = glm::dot(pos1, sideNormal1);
        float side2 = glm::dot(pos1, sideNormal2);
        negSide1 = -side1 + h1.y;
        posSide1 = side1 + h1.y;
        negSide2 = -side2 + h1.z;
        posSide2 = side2 + h1.z;
        negEdge1 = FACE3;
        posEdge1 = FACE5;
        negEdge2 = FACE2;
        posEdge2 = FACE4;
        ComputeIncidentFace(incidentFace, h2, pos2, Rot2, frontNormal);
        break;
    }

    case FACE_A_Y:
    {
        frontNormal = normal;
        front = glm::dot(pos1, frontNormal) + h1.y;
        sideNormal1 = Rot1[0];
        sideNormal2 = Rot1[2];
        float side1 = glm::dot(pos1, sideNormal1);
        float side2 = glm::dot(pos1, sideNormal2);
        negSide1 = -side1 + h1.x;
        posSide1 = side1 + h1.x;
        negSide2 = -side2 + h1.z;
        posSide2 = side2 + h1.z;
        negEdge1 = FACE1;
        posEdge1 = FACE6;
        negEdge2 = FACE2;
        posEdge2 = FACE4;
        ComputeIncidentFace(incidentFace, h2, pos2, Rot2, frontNormal);
        break;
    }

    case FACE_A_Z:
    {
        frontNormal = normal;
        front = glm::dot(pos1, frontNormal) + h1.z;
        sideNormal1 = Rot1[0];
        sideNormal2 = Rot1[1];
        float side1 = glm::dot(pos1, sideNormal1);
        float side2 = glm::dot(pos1, sideNormal2);
        negSide1 = -side1 + h1.x;
        posSide1 = side1 + h1.x;
        negSide2 = -side2 + h1.y;
        posSide2 = side2 + h1.y;
        negEdge1 = FACE1;
        posEdge1 = FACE6;
        negEdge2 = FACE3;
        posEdge2 = FACE5;
        ComputeIncidentFace(incidentFace, h2, pos2, Rot2, frontNormal);
        break;
    }

    case FACE_B_X:
    {
        frontNormal = normal;
        front = glm::dot(pos2, frontNormal) + h2.x;
        sideNormal1 = Rot2[1];
        sideNormal2 = Rot2[2];
        float side1 = glm::dot(pos2, sideNormal1);
        float side2 = glm::dot(pos2, sideNormal2);
        negSide1 = -side1 + h2.y;
        posSide1 = side1 + h2.y;
        negSide2 = -side2 + h2.z;
        posSide2 = side2 + h2.z;
        negEdge1 = FACE3;
        posEdge1 = FACE5;
        negEdge2 = FACE2;
        posEdge2 = FACE4;
        ComputeIncidentFace(incidentFace, h1, pos1, Rot1, frontNormal);
        break;
    }

    case FACE_B_Y:
    {
        frontNormal = normal;
        front = glm::dot(pos2, frontNormal) + h2.y;
        sideNormal1 = Rot2[0];
        sideNormal2 = Rot2[2];
        float side1 = glm::dot(pos2, sideNormal1);
        float side2 = glm::dot(pos2, sideNormal2);
        negSide1 = -side1 + h2.x;
        posSide1 = side1 + h2.x;
        negSide2 = -side2 + h2.z;
        posSide2 = side2 + h2.z;
        negEdge1 = FACE1;
        posEdge1 = FACE6;
        negEdge2 = FACE2;
        posEdge2 = FACE4;
        ComputeIncidentFace(incidentFace, h1, pos1, Rot1, frontNormal);
        break;
    }

    case FACE_B_Z:
    {
        frontNormal = normal;
        front = glm::dot(pos2, frontNormal) + h2.z;
        sideNormal1 = Rot2[0];
        sideNormal2 = Rot2[1];
        float side1 = glm::dot(pos2, sideNormal1);
        float side2 = glm::dot(pos2, sideNormal2);
        negSide1 = -side1 + h2.x;
        posSide1 = side1 + h2.x;
        negSide2 = -side2 + h2.y;
        posSide2 = side2 + h2.y;
        negEdge1 = FACE1;
        posEdge1 = FACE6;
        negEdge2 = FACE3;
        posEdge2 = FACE5;
        ComputeIncidentFace(incidentFace, h1, pos1, Rot1, frontNormal);
        break;
    }
    }

    ClipVertex3D clipPoints1[4];
    ClipVertex3D clipPoints2[4];
    int np;

    // Clip to box side 1
    np = ClipSegmentToPlane(clipPoints1, incidentFace, -sideNormal1, negSide1, negEdge1);
    if (np < 2)
        return 0;

    // Clip to box side 2
    np = ClipSegmentToPlane(clipPoints2, clipPoints1, sideNormal1, posSide1, posEdge1);
    if (np < 2)
        return 0;

    ClipVertex3D clipPoints3[4];

    // Clip to box side 3
    np = ClipSegmentToPlane(clipPoints3, clipPoints2, -sideNormal2, negSide2, negEdge2);
    if (np < 2)
        return 0;

    ClipVertex3D clipPoints4[4];

    // Clip to box side 4
    np = ClipSegmentToPlane(clipPoints4, clipPoints3, sideNormal2, posSide2, posEdge2);
    if (np < 2)
        return 0;

    // Now clipPoints4 contains the clipping points.
    // Due to roundoff, it is possible that clipping removes all points.
    int numContacts = 0;
    for (int i = 0; i < 4; ++i)
    {
        float separation = glm::dot(frontNormal, clipPoints4[i].v) - front;
        if (separation <= 0)
        {
            contacts[numContacts].separation = separation;
            contacts[numContacts].normal = normal;
            // slide contact point onto reference face (easy to cull)
            contacts[numContacts].position = clipPoints4[i].v - separation * frontNormal;
            contacts[numContacts].feature = clipPoints4[i].fp;

            if (axis == FACE_B_X || axis == FACE_B_Y || axis == FACE_B_Z)
            {
                Flip3D(contacts[numContacts].feature);
            }

            ++numContacts;
        }
    }

    return numContacts;
}