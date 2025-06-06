
#ifndef RIGIDDYNAMICS_BODIES_CONTACTS_CONTACT_H_
#define RIGIDDYNAMICS_BODIES_CONTACTS_CONTACT_H_

#include "../rigid_body.h"

namespace rigid_dynamics::bodies::contacts {

struct Contact {
 private:
  std::shared_ptr<RigidBody> body1;
  std::shared_ptr<RigidBody> body2;
  glm::dvec3 contact_point;
  glm::dvec3 contact_normal;  // from body2 to body1

  [[nodiscard]] double GetRelativeVelocity() const;
  void HandleRest() const;
  void HandleCollision(double relative_velocity) const;

 public:
  Contact(const std::shared_ptr<RigidBody>& body1,
          const std::shared_ptr<RigidBody>& body2,
          const glm::dvec3& contact_point, const glm::dvec3& contact_normal);
  void Handle(double collision_threshold);
};

}  // namespace rigid_dynamics::bodies::contacts

#endif  // RIGIDDYNAMICS_BODIES_CONTACTS_CONTACT_H_
