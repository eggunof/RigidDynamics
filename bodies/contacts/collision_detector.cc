
#include "collision_detector.h"

std::vector<Contact> CollisionDetector::DetectCollisions(
    const std::vector<RigidBody *> &bodies) {
  std::vector<Contact> contacts;
  for (size_t i = 0; i < bodies.size(); ++i) {
    for (size_t j = i + 1; j < bodies.size(); ++j) {
      Contact contact{};
      if (auto *sphereA = dynamic_cast<RigidSphere *>(bodies[i])) {
        if (auto *sphereB = dynamic_cast<RigidSphere *>(bodies[j])) {
          if (CheckCollision(*sphereA, *sphereB, contact))
            contacts.push_back(contact);
        } else if (auto *cubeB = dynamic_cast<RigidCube *>(bodies[j])) {
          if (CheckCollision(*cubeB, *sphereA, contact))
            contacts.push_back(contact);
        }
      } else if (auto *cubeA = dynamic_cast<RigidCube *>(bodies[i])) {
        if (auto *sphereB = dynamic_cast<RigidSphere *>(bodies[j])) {
          if (CheckCollision(*cubeA, *sphereB, contact))
            contacts.push_back(contact);
        } else if (auto *cubeB = dynamic_cast<RigidCube *>(bodies[j])) {
          if (CheckCollision(*cubeA, *cubeB, contact))
            contacts.push_back(contact);
        }
      }
    }
  }
  return contacts;
}

bool CollisionDetector::CheckCollision(const RigidSphere &sphereA,
                                       const RigidSphere &sphereB,
                                       Contact &contact) {
  glm::dvec3 diff = sphereB.position - sphereA.position;
  double distSq = glm::dot(diff, diff);
  double radiusSum = sphereA.radius + sphereB.radius;
  if (distSq < radiusSum * radiusSum) {
    double dist = sqrt(distSq);
    contact.n = dist > 0.0 ? diff / dist : glm::dvec3(1.0, 0.0, 0.0);
    contact.p = sphereA.position + contact.n * sphereA.radius;
    contact.vf = true;
    contact.a = (RigidBody *)&sphereA;
    contact.b = (RigidBody *)&sphereB;
    return true;
  }
  return false;
}

bool CollisionDetector::CheckCollision(const RigidCube &cubeA,
                                       const RigidCube &cubeB,
                                       Contact &contact) {
  return SATCollisionTest(cubeA, cubeB, contact);
}

bool CollisionDetector::CheckCollision(const RigidCube &cube,
                                       const RigidSphere &sphere,
                                       Contact &contact) {
  glm::dvec3 closestPoint = glm::clamp(
      sphere.position, cube.position - glm::dvec3(cube.edge_length / 2.0),
      cube.position + glm::dvec3(cube.edge_length / 2.0));
  glm::dvec3 diff = sphere.position - closestPoint;
  double distSq = glm::dot(diff, diff);
  if (distSq < sphere.radius * sphere.radius) {
    double dist = sqrt(distSq);
    contact.n = dist > 0.0 ? diff / dist : glm::dvec3(1.0, 0.0, 0.0);
    contact.p = closestPoint;
    contact.vf = true;
    contact.a = (RigidBody *)&cube;
    contact.b = (RigidBody *)&sphere;
    return true;
  }
  return false;
}

std::array<glm::dvec3, 3> CollisionDetector::GetCubeAxes(
    const RigidCube &cube) {
  glm::dmat3 rotMat = glm::mat3_cast(cube.rotation);
  return {glm::dvec3(rotMat[0]), glm::dvec3(rotMat[1]), glm::dvec3(rotMat[2])};
}

bool CollisionDetector::SATCollisionTest(const RigidCube &cubeA,
                                         const RigidCube &cubeB,
                                         Contact &contact) {
  std::array<glm::dvec3, 3> axesA = GetCubeAxes(cubeA);
  std::array<glm::dvec3, 3> axesB = GetCubeAxes(cubeB);
  std::vector<glm::dvec3> testAxes;

  testAxes.insert(testAxes.end(), axesA.begin(), axesA.end());
  testAxes.insert(testAxes.end(), axesB.begin(), axesB.end());
  for (const auto &axisA : axesA) {
    for (const auto &axisB : axesB) {
      testAxes.push_back(glm::cross(axisA, axisB));
    }
  }

  for (const auto &axis : testAxes) {
    if (glm::length(axis) < 1e-6) continue;
    glm::dvec3 normalizedAxis = glm::normalize(axis);
    double minA, maxA, minB, maxB;

    minA = maxA = glm::dot(cubeA.position, normalizedAxis);
    minB = maxB = glm::dot(cubeB.position, normalizedAxis);

    for (int i = -1; i <= 1; i += 2) {
      for (int j = -1; j <= 1; j += 2) {
        for (int k = -1; k <= 1; k += 2) {
          glm::dvec3 cornerA = cubeA.position +
                               (double)i * cubeA.edge_length / 2.0 * axesA[0] +
                               (double)j * cubeA.edge_length / 2.0 * axesA[1] +
                               (double)k * cubeA.edge_length / 2.0 * axesA[2];
          double projectionA = glm::dot(cornerA, normalizedAxis);
          minA = std::min(minA, projectionA);
          maxA = std::max(maxA, projectionA);
        }
      }
    }
    if (maxA < minB || maxB < minA) return false;
  }
  contact.vf = false;
  return true;
}