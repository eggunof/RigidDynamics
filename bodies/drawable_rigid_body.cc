
#include "drawable_rigid_body.h"

namespace rigid_dynamics::bodies {

DrawableRigidBody::DrawableRigidBody(
    double mass, const glm::dmat3& body_inertia, double restitution,
    const glm::dvec3& position, const glm::dquat& rotation,
    const glm::dvec3& momentum, const glm::dvec3& angular_momentum)
    : RigidBody(mass, body_inertia, restitution, position, rotation, momentum,
                angular_momentum) {}

DrawableRigidBody::DrawableRigidBody(double restitution,
                                     const glm::dvec3& position,
                                     const glm::dquat& rotation)
    : RigidBody(restitution, position, rotation) {}

}  // namespace rigid_dynamics::bodies