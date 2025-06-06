#ifndef RIGIDDYNAMICS_COLLISION_DETECTOR_H_
#define RIGIDDYNAMICS_COLLISION_DETECTOR_H_

#include <array>
#include <vector>

#include "../rigid_body_old.h"
#include "../rigid_cube.h"
#include "../rigid_sphere.h"

struct Contact {
  RigidBody *a;  /* body containing vertex */
  RigidBody *b;  /* body containing face */
  glm::dvec3 p;  /* world-space vertex location */
  glm::dvec3 n;  /* outwards pointing normal of face */
  glm::dvec3 ea; /* edge direction for A */
  glm::dvec3 eb; /* edge direction for B */
  bool vf;       /* true if vertex/face contact */
};

class CollisionDetector {
 public:
  static std::vector<Contact> DetectCollisions(
      const std::vector<RigidBody *> &bodies);
  static bool CheckCollision(const RigidSphere &sphereA,
                             const RigidSphere &sphereB, Contact &contact);
  static bool CheckCollision(const RigidCube &cubeA, const RigidCube &cubeB,
                             Contact &contact);
  static bool CheckCollision(const RigidCube &cube, const RigidSphere &sphere,
                             Contact &contact);

 private:
  static bool SATCollisionTest(const RigidCube &cubeA, const RigidCube &cubeB,
                               Contact &contact);
  static std::array<glm::dvec3, 3> GetCubeAxes(const RigidCube &cube);
};


#endif  // RIGIDDYNAMICS_COLLISION_DETECTOR_H_
