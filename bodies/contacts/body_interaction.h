
#ifndef RIGIDDYNAMICS_BODIES_CONTACTS_BODY_INTERACTION_H_
#define RIGIDDYNAMICS_BODIES_CONTACTS_BODY_INTERACTION_H_

#include <vector>

#include "../rigid_body.h"
#include "contact.h"

namespace rigid_dynamics::bodies::contacts {

struct Projection {
  double min;
  double max;

  bool IsOverlapsWith(const Projection& other) const {
    return min <= other.max && max >= other.min;
  }

  [[nodiscard]] double GetOverlap(const Projection& other) const;
};

struct BodyInteraction {
  std::shared_ptr<RigidBody> body1;
  std::shared_ptr<RigidBody> body2;

  BodyInteraction(const std::shared_ptr<RigidBody>& body1,
                  const std::shared_ptr<RigidBody>& body2);

  void ProcessContacts(double collision_threshold);

 private:
  static std::vector<std::shared_ptr<Contact>> GetSphereSphereContacts(
      const std::shared_ptr<RigidSphere>& sphere1,
      const std::shared_ptr<RigidSphere>& sphere2);
  static std::vector<std::shared_ptr<Contact>> GetBoxSphereContacts(
      const std::shared_ptr<RigidBox>& box,
      const std::shared_ptr<RigidSphere>& sphere);
  static std::vector<std::shared_ptr<Contact>> GetBoxBoxContacts(
      const std::shared_ptr<RigidBox>& box1,
      const std::shared_ptr<RigidBox>& box2);
};

}  // namespace rigid_dynamics::bodies::contacts

#endif  // RIGIDDYNAMICS_BODIES_CONTACTS_BODY_INTERACTION_H_
