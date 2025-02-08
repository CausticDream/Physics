#include "box2d-lite/Body3D.h"

Body3D::Body3D()
{
	position = glm::vec3(0.0f, 0.0f, 0.0f);
	rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
	velocity = glm::vec3(0.0f, 0.0f, 0.0f);
	angularVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
	force = glm::vec3(0.0f, 0.0f, 0.0f);
	torque = glm::vec3(0.0f, 0.0f, 0.0f);
	friction = 0.2f;
	mass = FLT_MAX;
	invMass = 0.0f;

	width = glm::vec3(1.0f, 1.0f, 1.0f);
	I = glm::mat3(1.0f);
	invI = glm::mat3(0.0f);
}

void Body3D::Set(const glm::vec3& w, float m)
{
	position = glm::vec3(0.0f, 0.0f, 0.0f);
	rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
	velocity = glm::vec3(0.0f, 0.0f, 0.0f);
	angularVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
	force = glm::vec3(0.0f, 0.0f, 0.0f);
	torque = glm::vec3(0.0f, 0.0f, 0.0f);
	friction = 0.2f;

	width = w;
	mass = m;

	if (mass < FLT_MAX)
	{
		invMass = 1.0f / mass;
		I = glm::mat3(glm::vec3(mass * (width.y * width.y + width.z * width.z) / 12.0f, 0.0f, 0.0f),
					  glm::vec3(0.0f, mass * (width.x * width.x + width.z * width.z) / 12.0f, 0.0f),
					  glm::vec3(0.0f, 0.0f, mass * (width.x * width.x + width.y * width.y) / 12.0f));
		invI = glm::inverse(I);
	}
	else
	{
		invMass = 0.0f;
		I = glm::mat3(1.0f);
		invI = glm::mat3(0.0f);
	}
}
