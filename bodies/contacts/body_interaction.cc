
#include "body_interaction.h"

#include <utility>

#include "../rigid_box.h"
#include "../rigid_sphere.h"

namespace rigid_dynamics::bodies::contacts {

double Projection::GetOverlap(const Projection& other) const {
  return std::max(0.0, std::min(max, other.max) - std::max(min, other.min));
}

BodyInteraction::BodyInteraction(const std::shared_ptr<RigidBody>& body1,
                                 const std::shared_ptr<RigidBody>& body2)
    : body1(body1), body2(body2) {}

void BodyInteraction::ProcessContacts(double collision_threshold) {
  std::vector<std::shared_ptr<Contact>> contacts;
  switch (body1->GetShapeType()) {
    case RigidBody::ShapeType::BOX:
      switch (body2->GetShapeType()) {
        case RigidBody::ShapeType::BOX:
          contacts =
              GetBoxBoxContacts(std::dynamic_pointer_cast<RigidBox>(body1),
                                std::dynamic_pointer_cast<RigidBox>(body2));
          break;
        case RigidBody::ShapeType::SPHERE:
          contacts = GetBoxSphereContacts(
              std::dynamic_pointer_cast<RigidBox>(body1),
              std::dynamic_pointer_cast<RigidSphere>(body2));
          break;
      }
      break;
    case RigidBody::ShapeType::SPHERE:
      switch (body2->GetShapeType()) {
        case RigidBody::ShapeType::BOX:
          contacts = GetBoxSphereContacts(
              std::dynamic_pointer_cast<RigidBox>(body2),
              std::dynamic_pointer_cast<RigidSphere>(body1));
          break;
        case RigidBody::ShapeType::SPHERE:
          contacts = GetSphereSphereContacts(
              std::dynamic_pointer_cast<RigidSphere>(body1),
              std::dynamic_pointer_cast<RigidSphere>(body2));
          break;
      }
      break;
  }

  for (auto& contact : contacts) {
    contact->Handle(collision_threshold);
  }
}

std::vector<std::shared_ptr<Contact>> BodyInteraction::GetSphereSphereContacts(
    const std::shared_ptr<RigidSphere>& sphere1,
    const std::shared_ptr<RigidSphere>& sphere2) {
  const glm::dvec3& sphere1_position = sphere1->GetPosition();
  const glm::dvec3& sphere2_position = sphere2->GetPosition();
  double sphere1_radius = sphere1->GetRadius();

  glm::dvec3 delta = sphere1_position - sphere2_position;
  double distance = glm::length(delta);
  if (distance == 0) delta = {0.01, 0.01, 0.01};

  double radius_sum = sphere1_radius + sphere2->GetRadius();
  if (distance < radius_sum) {
    glm::dvec3 collision_normal = glm::normalize(delta);
    glm::dvec3 contact_point =
        sphere1_position + collision_normal * sphere1_radius;

    return {std::make_shared<contacts::Contact>(sphere1, sphere2, contact_point,
                                                collision_normal)};
  }
  return {};
}

std::vector<std::shared_ptr<Contact>> BodyInteraction::GetBoxSphereContacts(
    const std::shared_ptr<RigidBox>& box,
    const std::shared_ptr<RigidSphere>& sphere) {
  const glm::dvec3& sphere_position = sphere->GetPosition();
  double sphere_radius = sphere->GetRadius();

  glm::dvec3 closest_point = box->GetClosestPoint(sphere_position);

  glm::dvec3 delta = sphere_position - closest_point;
  double distance = glm::length(delta);
  if (distance == 0) {
    delta = sphere_position - box->GetPosition();
    distance = glm::length(delta);
    if (distance == 0) delta = {0.01, 0.01, 0.01};
  }

  if (distance < sphere_radius) {
    glm::dvec3 collision_normal = glm::normalize(delta);
    glm::dvec3 contact_point = closest_point + collision_normal * sphere_radius;

    return {std::make_shared<contacts::Contact>(sphere, box, contact_point,
                                                collision_normal)};
  }
  return {};
}

std::vector<std::shared_ptr<Contact>> BodyInteraction::GetBoxBoxContacts(
    const std::shared_ptr<RigidBox>& box1,
    const std::shared_ptr<RigidBox>& box2) {
  std::vector<std::shared_ptr<Contact>> contacts;

  auto vertices1 = box1->GetVertices();
  auto vertices2 = box2->GetVertices();
  auto axes1 = box1->GetAxes();
  auto axes2 = box2->GetAxes();

  double min_penetration = std::numeric_limits<double>::max();
  glm::dvec3 collision_normal(0.0);

  auto projectVertices = [](const std::array<glm::dvec3, 8>& verts,
                            const glm::dvec3& axis) -> Projection {
    double min_proj = glm::dot(verts[0], axis);
    double max_proj = min_proj;
    for (int i = 1; i < 8; ++i) {
      double proj = glm::dot(verts[i], axis);
      if (proj < min_proj) min_proj = proj;
      if (proj > max_proj) max_proj = proj;
    }
    return Projection{min_proj, max_proj};
  };

  std::vector<glm::dvec3> test_axes;
  test_axes.reserve(15);
  for (int i = 0; i < 3; ++i) test_axes.push_back(axes1[i]);
  for (int i = 0; i < 3; ++i) test_axes.push_back(axes2[i]);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      glm::dvec3 axis = glm::cross(axes1[i], axes2[j]);
      if (glm::length(axis) > 1e-8) {
        test_axes.push_back(glm::normalize(axis));
      }
    }
  }

  for (const auto& axis : test_axes) {
    Projection proj1 = projectVertices(vertices1, axis);
    Projection proj2 = projectVertices(vertices2, axis);

    if (!proj1.IsOverlapsWith(proj2)) {
      return {};
    } else {
      double overlap = proj1.GetOverlap(proj2);
      if (overlap < min_penetration) {
        min_penetration = overlap;
        collision_normal = axis;
        if (glm::dot(collision_normal,
                     box1->GetPosition() - box2->GetPosition()) < 0.0) {
          collision_normal = -collision_normal;
        }
      }
    }
  }

  for (const auto& v : vertices1) {
    if (box2->IsPointInside(v)) {
      contacts.push_back(
          std::make_shared<contacts::Contact>(box1, box2, v, collision_normal));
    }
  }

  for (const auto& v : vertices2) {
    if (box1->IsPointInside(v)) {
      contacts.push_back(
          std::make_shared<contacts::Contact>(box1, box2, v, collision_normal));
    }
  }

  if (contacts.empty()) {
    glm::dvec3 contact_point =
        (box1->GetPosition() + box2->GetPosition()) * 0.5;
    contacts.push_back(std::make_shared<contacts::Contact>(
        box1, box2, contact_point, collision_normal));
  }

  return contacts;
}

}  // namespace rigid_dynamics::bodies::contacts