#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>

#include "imgui/imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"

#define GLFW_INCLUDE_NONE
#include "GLFW/glfw3.h"
#include "glad/glad.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/random.hpp>

#include "Body.h"
#include "Joint.h"
#include "World.h"

namespace
{
    GLFWwindow* mainWindow = NULL;

    int width = 1920;
    int height = 1080;
    float zoom = 10.0f;
    float pan_y = 8.0f;
    int demoIndex = 0;
    float timeStep = 1.0f / 60.0f;

    Body bodies[200];
    Joint joints[100];

    Body* bomb = NULL;

    int numBodies = 0;
    int numJoints = 0;

    glm::vec3 gravity(0.0f, -9.81f, 0.0f);
    int iterations = 10;
    World world(gravity, iterations);
} // namespace

static void glfwErrorCallback(int error, const char* description)
{
    printf("GLFW error %d: %s\n", error, description);
}

static void DrawText(int x, int y, const char* string)
{
    ImVec2 p;
    p.x = float(x);
    p.y = float(y);
    ImGui::Begin("Overlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
    ImGui::SetCursorPos(p);
    ImGui::TextColored(ImColor(230, 153, 153, 255), "%s", string);
    ImGui::End();
}

static void DrawShape(Body* body, Shape* shape)
{
    if (shape->GetType() == ShapeType::Box)
    {
        ShapeBox* shapeBox = static_cast<ShapeBox*>(shape);

        glm::mat3 R = glm::mat3_cast(body->rotation);

        glm::vec3 v1 = body->position + R * glm::vec3(-shapeBox->halfSize.x, -shapeBox->halfSize.y, -shapeBox->halfSize.z);
        glm::vec3 v2 = body->position + R * glm::vec3(shapeBox->halfSize.x, -shapeBox->halfSize.y, -shapeBox->halfSize.z);
        glm::vec3 v3 = body->position + R * glm::vec3(shapeBox->halfSize.x, shapeBox->halfSize.y, -shapeBox->halfSize.z);
        glm::vec3 v4 = body->position + R * glm::vec3(-shapeBox->halfSize.x, shapeBox->halfSize.y, -shapeBox->halfSize.z);
        glm::vec3 v5 = body->position + R * glm::vec3(-shapeBox->halfSize.x, -shapeBox->halfSize.y, shapeBox->halfSize.z);
        glm::vec3 v6 = body->position + R * glm::vec3(shapeBox->halfSize.x, -shapeBox->halfSize.y, shapeBox->halfSize.z);
        glm::vec3 v7 = body->position + R * glm::vec3(shapeBox->halfSize.x, shapeBox->halfSize.y, shapeBox->halfSize.z);
        glm::vec3 v8 = body->position + R * glm::vec3(-shapeBox->halfSize.x, shapeBox->halfSize.y, shapeBox->halfSize.z);

        if (body == bomb)
        {
            glColor3f(0.4f, 0.9f, 0.4f);
        }
        else
        {
            glColor3f(0.8f, 0.8f, 0.9f);
        }

        glBegin(GL_LINES);

        // bottom
        glVertex3f(v1.x, v1.y, v1.z);
        glVertex3f(v2.x, v2.y, v2.z);
        glVertex3f(v2.x, v2.y, v2.z);
        glVertex3f(v3.x, v3.y, v3.z);
        glVertex3f(v3.x, v3.y, v3.z);
        glVertex3f(v4.x, v4.y, v4.z);
        glVertex3f(v4.x, v4.y, v4.z);
        glVertex3f(v1.x, v1.y, v1.z);

        // top
        glVertex3f(v5.x, v5.y, v5.z);
        glVertex3f(v6.x, v6.y, v6.z);
        glVertex3f(v6.x, v6.y, v6.z);
        glVertex3f(v7.x, v7.y, v7.z);
        glVertex3f(v7.x, v7.y, v7.z);
        glVertex3f(v8.x, v8.y, v8.z);
        glVertex3f(v8.x, v8.y, v8.z);
        glVertex3f(v5.x, v5.y, v5.z);

        // close the cube
        glVertex3f(v1.x, v1.y, v1.z);
        glVertex3f(v5.x, v5.y, v5.z);
        glVertex3f(v2.x, v2.y, v2.z);
        glVertex3f(v6.x, v6.y, v6.z);
        glVertex3f(v3.x, v3.y, v3.z);
        glVertex3f(v7.x, v7.y, v7.z);
        glVertex3f(v4.x, v4.y, v4.z);
        glVertex3f(v8.x, v8.y, v8.z);

        glEnd();
    }
}

static void DrawJoint(Joint* joint)
{
    Body* b1 = joint->body1;
    Body* b2 = joint->body2;

    glm::vec3 x1 = b1->position;
    glm::vec3 p1 = x1 + b1->rotation * joint->localAnchor1;

    glm::vec3 x2 = b2->position;
    glm::vec3 p2 = x2 + b2->rotation * joint->localAnchor2;

    glColor3f(0.5f, 0.5f, 0.8f);
    glBegin(GL_LINES);
    glVertex3f(x1.x, x1.y, x1.z);
    glVertex3f(p1.x, p1.y, p1.z);
    glVertex3f(x2.x, x2.y, x2.z);
    glVertex3f(p2.x, p2.y, p2.z);
    glEnd();
}

static void LaunchBomb()
{
    if (!bomb)
    {
        bomb = bodies + numBodies;
        static_cast<ShapeBox*>(bomb->shapes[0])->Set(glm::vec3(0.5f, 0.5f, 0.5f));
        bomb->SetMass(50.0f);
        world.Add(bomb);
        ++numBodies;
    }

    bomb->position = glm::vec3(glm::linearRand(-15.0f, 15.0f), 15.0f, 0.0f);
    bomb->rotation = glm::quat(glm::vec3(0.0f, 0.0f, glm::linearRand(-1.5f, 1.5f)));
    bomb->velocity = -1.5f * bomb->position;
    bomb->angularVelocity = glm::vec3(0.0f, 0.0f, glm::linearRand(-20.0f, 20.0f));
}

// Single box
static void Demo1(Body* b, Joint* j)
{
    static_cast<ShapeBox*>(b->shapes[0])->Set(glm::vec3(50.0f, 10.0f, 10.0f));
    b->SetMass(FLT_MAX);
    b->position = glm::vec3(0.0f, -static_cast<ShapeBox*>(b->shapes[0])->halfSize.y, 0.0f);
    world.Add(b);
    ++b;
    ++numBodies;

    static_cast<ShapeBox*>(b->shapes[0])->Set(glm::vec3(0.5f, 0.5f, 0.5f));
    b->SetMass(200.0f);
    b->position = glm::vec3(0.0f, 4.0f, 0.0f);
    world.Add(b);
    ++b;
    ++numBodies;
}

// A simple pendulum
static void Demo2(Body* b, Joint* j)
{
    Body* b1 = b + 0;
    static_cast<ShapeBox*>(b1->shapes[0])->Set(glm::vec3(50.0f, 10.0f, 10.0f));
    b1->SetMass(FLT_MAX);
    b1->position = glm::vec3(0.0f, -static_cast<ShapeBox*>(b1->shapes[0])->halfSize.y, 0.0f);
    b1->rotation = glm::quat(glm::vec3(0.0f, 0.0f, 0.0f));
    world.Add(b1);

    Body* b2 = b + 1;
    static_cast<ShapeBox*>(b2->shapes[0])->Set(glm::vec3(0.5f, 0.5f, 0.5f));
    b2->SetMass(100.0f);
    b2->position = glm::vec3(9.0f, 11.0f, 0.0f);
    b2->rotation = glm::quat(glm::vec3(0.0f, 0.0f, 0.0f));
    world.Add(b2);

    numBodies += 2;

    j->Set(b1, b2, glm::vec3(0.0f, 11.0f, 0.0f));
    world.Add(j);

    numJoints += 1;
}

// Varying friction coefficients
static void Demo3(Body* b, Joint* j)
{
    static_cast<ShapeBox*>(b->shapes[0])->Set(glm::vec3(50.0f, 10.0f, 10.0f));
    b->SetMass(FLT_MAX);
    b->position = glm::vec3(0.0f, -static_cast<ShapeBox*>(b->shapes[0])->halfSize.y, 0.0f);
    world.Add(b);
    ++b;
    ++numBodies;

    static_cast<ShapeBox*>(b->shapes[0])->Set(glm::vec3(6.5f, 0.125f, 0.125f));
    b->SetMass(FLT_MAX);
    b->position = glm::vec3(-2.0f, 11.0f, 0.0f);
    b->rotation = glm::quat(glm::vec3(0.0f, 0.0f, -0.25f));
    world.Add(b);
    ++b;
    ++numBodies;

    static_cast<ShapeBox*>(b->shapes[0])->Set(glm::vec3(0.125f, 0.5f, 0.5f));
    b->SetMass(FLT_MAX);
    b->position = glm::vec3(5.25f, 9.5f, 0.0f);
    world.Add(b);
    ++b;
    ++numBodies;

    static_cast<ShapeBox*>(b->shapes[0])->Set(glm::vec3(6.5f, 0.125f, 0.125f));
    b->SetMass(FLT_MAX);
    b->position = glm::vec3(2.0f, 7.0f, 0.0f);
    b->rotation = glm::quat(glm::vec3(0.0f, 0.0f, 0.25f));
    world.Add(b);
    ++b;
    ++numBodies;

    static_cast<ShapeBox*>(b->shapes[0])->Set(glm::vec3(0.125f, 0.5f, 0.5f));
    b->SetMass(FLT_MAX);
    b->position = glm::vec3(-5.25f, 5.5f, 0.0f);
    world.Add(b);
    ++b;
    ++numBodies;

    static_cast<ShapeBox*>(b->shapes[0])->Set(glm::vec3(6.5f, 0.125f, 0.125f));
    b->SetMass(FLT_MAX);
    b->position = glm::vec3(-2.0f, 3.0f, 0.0f);
    b->rotation = glm::quat(glm::vec3(0.0f, 0.0f, -0.25f));
    world.Add(b);
    ++b;
    ++numBodies;

    float friction[5] = {0.75f, 0.5f, 0.35f, 0.1f, 0.0f};
    for (int i = 0; i < 5; ++i)
    {
        static_cast<ShapeBox*>(b->shapes[0])->Set(glm::vec3(0.25f, 0.25f, 0.25f));
        b->SetMass(25.0f);
        b->shapes[0]->material->staticFriction = friction[i];
        b->shapes[0]->material->dynamicFriction = friction[i];
        b->position = glm::vec3(-7.5f + 2.0f * i, 14.0f, 0.0f);
        world.Add(b);
        ++b;
        ++numBodies;
    }
}

// A vertical stack
static void Demo4(Body* b, Joint* j)
{
    static_cast<ShapeBox*>(b->shapes[0])->Set(glm::vec3(50.0f, 10.0f, 10.0f));
    b->SetMass(FLT_MAX);
    b->position = glm::vec3(0.0f, -static_cast<ShapeBox*>(b->shapes[0])->halfSize.y, 0.0f);
    b->rotation = glm::quat(glm::vec3(0.0f, 0.0f, 0.0f));
    world.Add(b);
    ++b;
    ++numBodies;

    for (int i = 0; i < 10; ++i)
    {
        static_cast<ShapeBox*>(b->shapes[0])->Set(glm::vec3(0.5f, 0.5f, 0.5f));
        b->SetMass(1.0f);
        float x = glm::linearRand(-0.1f, 0.1f);
        b->position = glm::vec3(x, 0.51f + 1.05f * i, 0.0f);
        world.Add(b);
        ++b;
        ++numBodies;
    }
}

// A pyramid
static void Demo5(Body* b, Joint* j)
{
    static_cast<ShapeBox*>(b->shapes[0])->Set(glm::vec3(50.0f, 10.0f, 10.0f));
    b->SetMass(FLT_MAX);
    b->position = glm::vec3(0.0f, -static_cast<ShapeBox*>(b->shapes[0])->halfSize.y, 0.0f);
    b->rotation = glm::quat(glm::vec3(0.0f, 0.0f, 0.0f));
    world.Add(b);
    ++b;
    ++numBodies;

    glm::vec3 x(-6.0f, 0.75f, 0.0f);
    glm::vec3 y;

    for (int i = 0; i < 12; ++i)
    {
        y = x;

        for (int j = i; j < 12; ++j)
        {
            static_cast<ShapeBox*>(b->shapes[0])->Set(glm::vec3(0.5f, 0.5f, 0.5f));
            b->SetMass(10.0f);
            b->position = y;
            world.Add(b);
            ++b;
            ++numBodies;

            y += glm::vec3(1.125f, 0.0f, 0.0f);
        }

        x += glm::vec3(0.5625f, 2.0f, 0.0f);
    }
}

// A teeter
static void Demo6(Body* b, Joint* j)
{
    Body* b1 = b + 0;
    static_cast<ShapeBox*>(b1->shapes[0])->Set(glm::vec3(50.0f, 10.0f, 10.0f));
    b1->SetMass(FLT_MAX);
    b1->position = glm::vec3(0.0f, -static_cast<ShapeBox*>(b1->shapes[0])->halfSize.y, 0.0f);
    world.Add(b1);

    Body* b2 = b + 1;
    static_cast<ShapeBox*>(b2->shapes[0])->Set(glm::vec3(6.0f, 0.125f, 0.125f));
    b2->SetMass(100.0f);
    b2->position = glm::vec3(0.0f, 1.0f, 0.0f);
    world.Add(b2);

    Body* b3 = b + 2;
    static_cast<ShapeBox*>(b3->shapes[0])->Set(glm::vec3(0.25f, 0.25f, 0.25f));
    b3->SetMass(25.0f);
    b3->position = glm::vec3(-5.0f, 2.0f, 0.0f);
    world.Add(b3);

    Body* b4 = b + 3;
    static_cast<ShapeBox*>(b4->shapes[0])->Set(glm::vec3(0.25f, 0.25f, 0.25f));
    b4->SetMass(25.0f);
    b4->position = glm::vec3(-5.5f, 2.0f, 0.0f);
    world.Add(b4);

    Body* b5 = b + 4;
    static_cast<ShapeBox*>(b5->shapes[0])->Set(glm::vec3(0.5f, 0.5f, 0.5f));
    b5->SetMass(100.0f);
    b5->position = glm::vec3(5.5f, 15.0f, 0.0f);
    world.Add(b5);

    numBodies += 5;

    j->Set(b1, b2, glm::vec3(0.0f, 1.0f, 0.0f));
    world.Add(j);

    numJoints += 1;
}

// A suspension bridge
static void Demo7(Body* b, Joint* j)
{
    static_cast<ShapeBox*>(b->shapes[0])->Set(glm::vec3(50.0f, 10.0f, 10.0f));
    b->SetMass(FLT_MAX);
    b->position = glm::vec3(0.0f, -static_cast<ShapeBox*>(b->shapes[0])->halfSize.y, 0.0f);
    b->rotation = glm::quat(glm::vec3(0.0f, 0.0f, 0.0f));
    world.Add(b);
    ++b;
    ++numBodies;

    const int numPlanks = 15;
    float mass = 50.0f;

    for (int i = 0; i < numPlanks; ++i)
    {
        static_cast<ShapeBox*>(b->shapes[0])->Set(glm::vec3(0.5f, 0.125f, 0.125f));
        b->SetMass(mass);
        b->position = glm::vec3(-8.5f + 1.25f * i, 5.0f, 0.0f);
        world.Add(b);
        ++b;
        ++numBodies;
    }

    // Tuning
    float frequencyHz = 2.0f;
    float dampingRatio = 0.7f;

    // frequency in radians
    float omega = 2.0f * glm::pi<float>() * frequencyHz;

    // damping coefficient
    float d = 2.0f * mass * dampingRatio * omega;

    // spring stifness
    float k = mass * omega * omega;

    // magic formulas
    float softness = 1.0f / (d + timeStep * k);
    float biasFactor = timeStep * k / (d + timeStep * k);

    for (int i = 0; i < numPlanks; ++i)
    {
        j->Set(bodies + i, bodies + i + 1, glm::vec3(-9.125f + 1.25f * i, 5.0f, 0.0f));
        j->softness = softness;
        j->biasFactor = biasFactor;

        world.Add(j);
        ++j;
        ++numJoints;
    }

    j->Set(bodies + numPlanks, bodies, glm::vec3(-9.125f + 1.25f * numPlanks, 5.0f, 0.0f));
    j->softness = softness;
    j->biasFactor = biasFactor;
    world.Add(j);
    ++j;
    ++numJoints;
}

// Dominos
static void Demo8(Body* b, Joint* j)
{
    Body* b1 = b;
    static_cast<ShapeBox*>(b->shapes[0])->Set(glm::vec3(50.0f, 10.0f, 10.0f));
    b->SetMass(FLT_MAX);
    b->position = glm::vec3(0.0f, -static_cast<ShapeBox*>(b->shapes[0])->halfSize.y, 0.0f);
    world.Add(b);
    ++b;
    ++numBodies;

    static_cast<ShapeBox*>(b->shapes[0])->Set(glm::vec3(6.0f, 0.25f, 0.25f));
    b->SetMass(FLT_MAX);
    b->position = glm::vec3(-1.5f, 10.0f, 0.0f);
    world.Add(b);
    ++b;
    ++numBodies;

    for (int i = 0; i < 10; ++i)
    {
        static_cast<ShapeBox*>(b->shapes[0])->Set(glm::vec3(0.1f, 1.0f, 1.0f));
        b->SetMass(10.0f);
        b->position = glm::vec3(-6.0f + 1.0f * i, 11.125f, 0.0f);
        b->shapes[0]->material->staticFriction = 0.1f;
        b->shapes[0]->material->dynamicFriction = 0.1f;
        world.Add(b);
        ++b;
        ++numBodies;
    }

    static_cast<ShapeBox*>(b->shapes[0])->Set(glm::vec3(7.0f, 0.25f, 0.25f));
    b->SetMass(FLT_MAX);
    b->position = glm::vec3(1.0f, 6.0f, 0.0f);
    b->rotation = glm::quat(glm::vec3(0.0f, 0.0f, 0.3f));
    world.Add(b);
    ++b;
    ++numBodies;

    Body* b2 = b;
    static_cast<ShapeBox*>(b->shapes[0])->Set(glm::vec3(0.25f, 1.5f, 1.5f));
    b->SetMass(FLT_MAX);
    b->position = glm::vec3(-7.0f, 4.0f, 0.0f);
    world.Add(b);
    ++b;
    ++numBodies;

    Body* b3 = b;
    static_cast<ShapeBox*>(b->shapes[0])->Set(glm::vec3(6.0f, 0.125f, 0.125f));
    b->SetMass(20.0f);
    b->position = glm::vec3(-0.9f, 1.0f, 0.0f);
    world.Add(b);
    ++b;
    ++numBodies;

    j->Set(b1, b3, glm::vec3(-2.0f, 1.0f, 0.0f));
    world.Add(j);
    ++j;
    ++numJoints;

    Body* b4 = b;
    static_cast<ShapeBox*>(b->shapes[0])->Set(glm::vec3(0.25f, 0.25f, 0.25f));
    b->SetMass(10.0f);
    b->position = glm::vec3(-10.0f, 15.0f, 0.0f);
    world.Add(b);
    ++b;
    ++numBodies;

    j->Set(b2, b4, glm::vec3(-7.0f, 15.0f, 0.0f));
    world.Add(j);
    ++j;
    ++numJoints;

    Body* b5 = b;
    static_cast<ShapeBox*>(b->shapes[0])->Set(glm::vec3(1.0f, 1.0f, 1.0f));
    b->SetMass(20.0f);
    b->position = glm::vec3(6.0f, 2.5f, 0.0f);
    b->shapes[0]->material->staticFriction = 0.1f;
    b->shapes[0]->material->dynamicFriction = 0.1f;
    world.Add(b);
    ++b;
    ++numBodies;

    j->Set(b1, b5, glm::vec3(6.0f, 2.6f, 0.0f));
    world.Add(j);
    ++j;
    ++numJoints;

    Body* b6 = b;
    static_cast<ShapeBox*>(b->shapes[0])->Set(glm::vec3(1.0f, 0.1f, 0.1f));
    b->SetMass(10.0f);
    b->position = glm::vec3(6.0f, 3.6f, 0.0f);
    world.Add(b);
    ++b;
    ++numBodies;

    j->Set(b5, b6, glm::vec3(7.0f, 3.5f, 0.0f));
    world.Add(j);
    ++j;
    ++numJoints;
}

// A multi-pendulum
static void Demo9(Body* b, Joint* j)
{
    static_cast<ShapeBox*>(b->shapes[0])->Set(glm::vec3(50.0f, 10.0f, 10.0f));
    b->SetMass(FLT_MAX);
    b->position = glm::vec3(0.0f, -static_cast<ShapeBox*>(b->shapes[0])->halfSize.y, 0.0f);
    b->rotation = glm::quat(glm::vec3(0.0f, 0.0f, 0.0f));
    world.Add(b);

    Body* b1 = b;
    ++b;
    ++numBodies;

    float mass = 10.0f;

    // Tuning
    float frequencyHz = 4.0f;
    float dampingRatio = 0.7f;

    // frequency in radians
    float omega = 2.0f * glm::pi<float>() * frequencyHz;

    // damping coefficient
    float d = 2.0f * mass * dampingRatio * omega;

    // spring stiffness
    float k = mass * omega * omega;

    // magic formulas
    float softness = 1.0f / (d + timeStep * k);
    float biasFactor = timeStep * k / (d + timeStep * k);

    const float y = 12.0f;

    for (int i = 0; i < 15; ++i)
    {
        glm::vec3 x(0.5f + i, y, 0.0f);
        static_cast<ShapeBox*>(b->shapes[0])->Set(glm::vec3(0.375f, 0.125f, 0.375f));
        b->SetMass(mass);
        b->position = x;
        b->rotation = glm::quat(glm::vec3(0.0f, 0.0f, 0.0f));
        world.Add(b);

        j->Set(b1, b, glm::vec3(float(i), y, 0.0f));
        j->softness = softness;
        j->biasFactor = biasFactor;
        world.Add(j);

        b1 = b;
        ++b;
        ++numBodies;
        ++j;
        ++numJoints;
    }
}

void (*demos[])(Body* b, Joint* j) = {Demo1, Demo2, Demo3, Demo4, Demo5, Demo6, Demo7, Demo8, Demo9};
const char* demoStrings[] = {
    "Demo 1: A Single Box",
    "Demo 2: Simple Pendulum",
    "Demo 3: Varying Friction Coefficients",
    "Demo 4: Randomized Stacking",
    "Demo 5: Pyramid Stacking",
    "Demo 6: A Teeter",
    "Demo 7: A Suspension Bridge",
    "Demo 8: Dominos",
    "Demo 9: Multi-pendulum"};

static void InitDemo(int index)
{
    for (int i = 0; i < numBodies; ++i)
    {
        Body* body = bodies + i;
        body->position = glm::vec3(0.0f, 0.0f, 0.0f);
        body->rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        body->velocity = glm::vec3(0.0f, 0.0f, 0.0f);
        body->angularVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
        body->force = glm::vec3(0.0f, 0.0f, 0.0f);
        body->torque = glm::vec3(0.0f, 0.0f, 0.0f);
        static_cast<ShapeBox*>(body->shapes[0])->material->staticFriction = 0.2f;
        static_cast<ShapeBox*>(body->shapes[0])->material->dynamicFriction = 0.2f;
        static_cast<ShapeBox*>(body->shapes[0])->material->restitution = 0.0f;
    }

    world.Clear();
    numBodies = 0;
    numJoints = 0;
    bomb = NULL;

    demoIndex = index;
    demos[index](bodies, joints);
}

static void Keyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (action != GLFW_PRESS)
    {
        return;
    }

    switch (key)
    {
        case GLFW_KEY_ESCAPE:
            // Quit
            glfwSetWindowShouldClose(mainWindow, GL_TRUE);
            break;

        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
            InitDemo(key - GLFW_KEY_1);
            break;

        case GLFW_KEY_SPACE:
            LaunchBomb();
            break;
    }
}

static void Reshape(GLFWwindow*, int w, int h)
{
    width = w;
    height = h > 0 ? h : 1;

    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glm::mat4 projection = glm::perspective(
        glm::radians(60.0f),
        float(width) / float(height),
        0.1f,
        100.0f);
    glLoadMatrixf(glm::value_ptr(projection));
}

int main(int, char**)
{
    glfwSetErrorCallback(glfwErrorCallback);

    if (glfwInit() == 0)
    {
        fprintf(stderr, "Failed to initialize GLFW\n");
        return -1;
    }

    mainWindow = glfwCreateWindow(width, height, "Physics", NULL, NULL);
    if (mainWindow == NULL)
    {
        fprintf(stderr, "Failed to open GLFW mainWindow.\n");
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(mainWindow);

    // Load OpenGL functions using glad
    int gladStatus = gladLoadGL();
    if (gladStatus == 0)
    {
        fprintf(stderr, "Failed to load OpenGL.\n");
        glfwTerminate();
        return -1;
    }

    glfwSwapInterval(1);
    glfwSetWindowSizeCallback(mainWindow, Reshape);
    glfwSetKeyCallback(mainWindow, Keyboard);

    float xscale, yscale;
    glfwGetWindowContentScale(mainWindow, &xscale, &yscale);
    float uiScale = xscale;

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsClassic();
    ImGui_ImplGlfw_InitForOpenGL(mainWindow, true);
    ImGui_ImplOpenGL2_Init();
    ImGuiIO& io = ImGui::GetIO();
    io.FontGlobalScale = uiScale;

    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glm::mat4 projection = glm::perspective(glm::radians(60.0f), float(width) / float(height), 0.1f, 100.0f);
    glLoadMatrixf(glm::value_ptr(projection));

    for (int i = 0; i < 200; ++i)
    {
        Body* body = bodies + i;
        ShapeBox* shapeBox = new ShapeBox;
        shapeBox->material = new Material;
        shapeBox->material->staticFriction = 0.2f;
        shapeBox->material->dynamicFriction = 0.2f;
        shapeBox->material->restitution = 0.0f;
        body->AddShape(shapeBox);
    }

    InitDemo(0);

    while (!glfwWindowShouldClose(mainWindow))
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ImGui_ImplOpenGL2_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Globally position text
        ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f));
        ImGui::Begin("Overlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
        ImGui::End();

        DrawText(5, 5, demoStrings[demoIndex]);
        DrawText(5, 35, "Keys: 1-9 Demos, Space to Launch the Bomb");

        glMatrixMode(GL_MODELVIEW);
        glm::mat4 view = glm::lookAt(glm::vec3(0.0f, 20.0f, 50.0f), glm::vec3(0.0f, 20.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        glLoadMatrixf(glm::value_ptr(view));

        world.Step(timeStep);

        for (int i = 0; i < numBodies; ++i)
        {
            Body* body = bodies + i;
            for (size_t s = 0; s < body->shapes.size(); ++s)
            {
                DrawShape(body, body->shapes[s]);
            }
        }

        for (int i = 0; i < numJoints; ++i)
        {
            DrawJoint(joints + i);
        }

        glPointSize(4.0f);
        glColor3f(1.0f, 0.0f, 0.0f);
        glBegin(GL_POINTS);

        for (const auto& iter : world.arbiters)
        {
            const Arbiter& arbiter = iter.second;
            for (int i = 0; i < arbiter.numContacts; ++i)
            {
                glm::vec3 p = arbiter.contacts[i].position;
                glVertex3f(p.x, p.y, p.z);
            }
        }

        glEnd();
        glPointSize(1.0f);

        ImGui::Render();
        ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

        glfwPollEvents();
        glfwSwapBuffers(mainWindow);
    }

    glfwTerminate();
    return 0;
}
