
#include "contact.h"

#include <iostream>

namespace rigid_dynamics::bodies ::contacts {

Contact::Contact(const std::shared_ptr<RigidBody>& body1,
                 const std::shared_ptr<RigidBody>& body2,
                 const glm::dvec3& contact_point,
                 const glm::dvec3& contact_normal)
    : body1(body1),
      body2(body2),
      contact_point(contact_point),
      contact_normal(contact_normal) {}

double Contact::GetRelativeVelocity() const {
  glm::dvec3 point_velocity1 = body1->GetPointVelocity(contact_point);
  glm::dvec3 point_velocity2 = body2->GetPointVelocity(contact_point);
  return glm::dot(contact_normal, point_velocity1 - point_velocity2);
}

void Contact::Handle(double collision_threshold) {
  if (body1->IsFixed() && body2->IsFixed()) {
    return;
  }

  double relative_velocity = GetRelativeVelocity();
  std::cout << relative_velocity << std::endl;
  if (relative_velocity > collision_threshold) {
    // Moving away
    return;
  }

  if (relative_velocity < -collision_threshold) {
    // Colliding contact
    HandleCollision(relative_velocity);
  } else {
    // Resting contact
    HandleRest();
  }
}

void Contact::HandleRest() const {
  if (!body1->IsFixed()) {
    body1->ApplyForce(glm::dot(-body1->GetAppliedForce(), contact_normal) *
                      contact_normal);
    body1->AddMomentum(glm::dot(-body1->GetMomentum(), contact_normal) *
                       contact_normal);
  }

  if (!body2->IsFixed()) {
    body2->ApplyForce(glm::dot(body2->GetAppliedForce(), contact_normal) *
                      -contact_normal);
    body2->AddMomentum(glm::dot(body2->GetMomentum(), contact_normal) *
                       -contact_normal);
  }
}

void Contact::HandleCollision(double relative_velocity) const {
  glm::dvec3 r1 = contact_point - body1->GetPosition();
  glm::dvec3 r2 = contact_point - body2->GetPosition();

  double restitution =
      std::min(body1->GetRestitution(), body2->GetRestitution());
  double numerator = -(1.0 + restitution) * relative_velocity;

  double term1 = 0, term2 = 0, term3 = 0, term4 = 0;
  if (!body1->IsFixed()) {
    term1 = 1.0 / body1->GetMass();
    term3 = glm::dot(
        contact_normal,
        glm::cross(glm::cross(r1, contact_normal) / body1->GetInertia(), r1));
  }
  if (!body2->IsFixed()) {
    term2 = 1.0 / body2->GetMass();
    term4 = glm::dot(
        contact_normal,
        glm::cross(glm::cross(r2, contact_normal) / body2->GetInertia(), r2));
  }

  double j = numerator / (term1 + term2 + term3 + term4);
  glm::dvec3 force = j * contact_normal;

  if (!body1->IsFixed()) {
    body1->AddMomentum(force);
    body1->AddAngularMomentum(glm::cross(r1, force));
  }

  if (!body2->IsFixed()) {
    body2->AddMomentum(-force);
    body2->AddAngularMomentum(-glm::cross(r2, force));
  }
}

}  // namespace rigid_dynamics::bodies::contacts