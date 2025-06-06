
#ifndef RIGIDDYNAMICS_BODIES_DRAWABLE_RIGID_BODY_H_
#define RIGIDDYNAMICS_BODIES_DRAWABLE_RIGID_BODY_H_

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "rigid_body.h"

namespace rigid_dynamics::bodies {

class DrawableRigidBody : public RigidBody {
 public:
  DrawableRigidBody(double mass, const glm::dmat3& body_inertia,
                    double restitution,
                    const glm::dvec3& position = {0.0, 0.0, 0.0},
                    const glm::dquat& rotation = {1.0, 0.0, 0.0, 0.0},
                    const glm::dvec3& momentum = {0.0, 0.0, 0.0},
                    const glm::dvec3& angular_momentum = {0.0, 0.0, 0.0});

  explicit DrawableRigidBody(double restitution,
                             const glm::dvec3& position = {0.0, 0.0, 0.0},
                             const glm::dquat& rotation = {1.0, 0.0, 0.0, 0.0});

  virtual void Draw() const = 0;
};

}  // namespace rigid_dynamics::bodies

#endif  // RIGIDDYNAMICS_BODIES_DRAWABLE_RIGID_BODY_H_
