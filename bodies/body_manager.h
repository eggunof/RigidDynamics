
#ifndef RIGIDDYNAMICS_COLLISION_MANAGER_H_
#define RIGIDDYNAMICS_COLLISION_MANAGER_H_

#include <memory>
#include <vector>

#include "contacts/body_interaction.h"
#include "drawable_rigid_body.h"

namespace rigid_dynamics::bodies {

#define GRAVITATIONAL_ACCELERATION 9.81

class BodyManager {
 public:
  BodyManager() = default;

  void AddBody(const std::shared_ptr<DrawableRigidBody>& body);
  void ClearBodies();
  void ApplyGravity() const;
  void ProcessInteractions();
  void IntegrateAll(double time, double h) const;
  void DrawAll() const;

  [[nodiscard]] size_t GetBodyCount() const;
  [[nodiscard]] std::shared_ptr<DrawableRigidBody> GetBody(size_t index) const;
  void RemoveBody(size_t i);

  const glm::dvec3& GetGravityAcceleration() const;
  void SetGravityAcceleration(const glm::dvec3& gravity_acceleration);

 private:
  glm::dvec3 gravity_acceleration_ = {0.0, -GRAVITATIONAL_ACCELERATION, 0.0};

  std::vector<std::shared_ptr<DrawableRigidBody>> bodies_;
  std::vector<std::shared_ptr<contacts::BodyInteraction>> interactions_;
};

}  // namespace rigid_dynamics::bodies

#endif  // RIGIDDYNAMICS_COLLISION_MANAGER_H_
