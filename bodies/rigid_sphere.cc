#include "rigid_sphere.h"

#include <GL/freeglut_std.h>
#include <GL/gl.h>

#include "rigid_box.h"

namespace rigid_dynamics::bodies {

/**
 * @brief Constructs a RigidSphere object.
 * @param mass Mass of the sphere.
 * @param radius Radius of the sphere.
 * @param restitution Coefficient of restitution.
 * @param position Initial position of the sphere.
 * @param rotation Initial rotation of the sphere.
 * @param momentum Initial linear momentum of the sphere.
 * @param angular_momentum Initial angular momentum of the sphere.
 * @param color Color of the sphere.
 */
RigidSphere::RigidSphere(double mass, double radius, double restitution,
                         const glm::dvec3& position, const glm::dquat& rotation,
                         const glm::dvec3& momentum,
                         const glm::dvec3& angular_momentum,
                         const glm::vec3& color)
    : DrawableRigidBody(mass, glm::dmat3(2 * mass * radius * radius / 5.0),
                        restitution, position, rotation, momentum,
                        angular_momentum),
      radius_(radius),
      color_(color) {}

/**
 * @brief Clones the current RigidSphere object.
 * @return A unique pointer to the cloned RigidSphere object.
 */
std::unique_ptr<RigidBody> RigidSphere::Clone() const {
  return std::make_unique<RigidSphere>(mass_, radius_, restitution_, position_,
                                       rotation_, momentum_, angular_momentum_,
                                       color_);
}

/**
 * @brief Gets the shape type of the rigid body.
 * @return The shape type of the rigid body (SPHERE).
 */
RigidBody::ShapeType RigidSphere::GetShapeType() const {
  return ShapeType::SPHERE;
}

/**
 * @brief Gets the radius of the sphere.
 * @return The radius of the sphere.
 */
double RigidSphere::GetRadius() const { return radius_; }

/**
 * @brief Draws the sphere using OpenGL.
 */
void RigidSphere::Draw() const {
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glTranslated(position_.x, position_.y, position_.z);
  glMultMatrixd(glm::value_ptr(glm::mat4_cast(rotation_)));

  glColor3fv(glm::value_ptr(color_));
  glutSolidSphere(radius_, SLICES, STACKS);

  glPopMatrix();
}

/**
 * @brief Calculates the inertia tensor for a sphere.
 * @param mass Mass of the sphere.
 * @param radius Radius of the sphere.
 * @return The inertia tensor of the sphere.
 */
glm::dmat3 RigidSphere::CalculateSphereInertia(double mass, double radius) {
  constexpr double kSphereInertiaCoefficient = 2.0 / 5.0;
  return glm::dmat3(mass * radius * radius * kSphereInertiaCoefficient);
}

}  // namespace rigid_dynamics::bodies