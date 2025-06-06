#include "rigid_box.h"

#include <GL/freeglut_std.h>
#include <GL/gl.h>

#include <glm/gtc/type_ptr.hpp>

#include "rigid_sphere.h"

namespace rigid_dynamics::bodies {

/**
 * @brief Creates a RigidBox object with mass.
 *
 * @param dimensions The dimensions of the box.
 * @param mass The mass of the box.
 * @param restitution The restitution coefficient of the box.
 * @param position The position of the box.
 * @param rotation The rotation of the box as a quaternion.
 * @param momentum The linear momentum of the box.
 * @param angular_momentum The angular momentum of the box.
 * @param face_colors The colors of the faces of the box.
 */
RigidBox::RigidBox(const glm::dvec3& dimensions, double mass,
                   double restitution, const glm::dvec3& position,
                   const glm::dquat& rotation, const glm::dvec3& momentum,
                   const glm::dvec3& angular_momentum,
                   const std::array<glm::vec3, 6>& face_colors)
    : DrawableRigidBody(mass, CalculateBoxInertia(mass, dimensions),
                        restitution, position, rotation, momentum,
                        angular_momentum),
      dimensions_(dimensions),
      face_colors(face_colors) {}

/**
 * @brief Creates a RigidBox object without mass.
 *
 * @param dimensions The dimensions of the box.
 * @param restitution The restitution coefficient of the box.
 * @param position The position of the box.
 * @param rotation The rotation of the box as a quaternion.
 * @param face_colors The colors of the faces of the box.
 */
RigidBox::RigidBox(const glm::dvec3& dimensions, double restitution,
                   const glm::dvec3& position, const glm::dquat& rotation,
                   const std::array<glm::vec3, 6>& face_colors)
    : DrawableRigidBody(restitution, position, rotation),
      dimensions_(dimensions),
      face_colors(face_colors) {}

/**
 * @brief Clones the RigidBox object.
 * @return A unique_ptr to a new RigidBox object.
 */
std::unique_ptr<RigidBody> RigidBox::Clone() const {
  if (fixed_) {
    return std::make_unique<RigidBox>(dimensions_, restitution_, position_,
                                      rotation_, face_colors);
  }
  return std::make_unique<RigidBox>(dimensions_, mass_, restitution_, position_,
                                    rotation_, momentum_, angular_momentum_,
                                    face_colors);
}

/**
 * @brief Gets the shape type of the rigid body.
 * @return The shape type of the rigid body.
 */
RigidBody::ShapeType RigidBox::GetShapeType() const { return ShapeType::BOX; }

/**
 * @brief Gets the dimensions of the rigid box.
 * @return The dimensions of the rigid box.
 */
const glm::dvec3& RigidBox::GetDimensions() const { return dimensions_; }

/**
 * @brief Calculates the vertices of the rigid box.
 * @return An array of vertices of the rigid box.
 */
std::array<glm::dvec3, 8> RigidBox::GetVertices() const {
  glm::dvec3 half_dimensions = dimensions_ * 0.5;
  std::array<glm::dvec3, 8> vertices = {
      glm::dvec3(-half_dimensions.x, -half_dimensions.y, -half_dimensions.z),
      glm::dvec3(half_dimensions.x, -half_dimensions.y, -half_dimensions.z),
      glm::dvec3(-half_dimensions.x, half_dimensions.y, -half_dimensions.z),
      glm::dvec3(half_dimensions.x, half_dimensions.y, -half_dimensions.z),
      glm::dvec3(-half_dimensions.x, -half_dimensions.y, half_dimensions.z),
      glm::dvec3(half_dimensions.x, -half_dimensions.y, half_dimensions.z),
      glm::dvec3(-half_dimensions.x, half_dimensions.y, half_dimensions.z),
      glm::dvec3(half_dimensions.x, half_dimensions.y, half_dimensions.z)};

  for (int i = 0; i < 8; ++i) {
    vertices[i] = position_ + rotation_ * vertices[i];
  }
  return vertices;
}

/**
 * @brief Gets the axes of the rigid box.
 * @return An array of axes of the rigid box.
 */
std::array<glm::dvec3, 3> RigidBox::GetAxes() const {
  return {rotation_ * glm::dvec3(1, 0, 0), rotation_ * glm::dvec3(0, 1, 0),
          rotation_ * glm::dvec3(0, 0, 1)};
}

/**
 * @brief Finds the closest point on the box to a given point.
 * @param point The point to find the closest point to.
 * @return The closest point on the box.
 */
glm::dvec3 RigidBox::GetClosestPoint(const glm::dvec3& point) const {
  glm::dvec3 local_point = glm::inverse(rotation_) * (point - position_);
  glm::dvec3 clamped =
      glm::clamp(local_point, -dimensions_ / 2.0, dimensions_ / 2.0);
  return position_ + rotation_ * clamped;
}

/**
 * @brief Checks if a point is inside the rigid box.
 * @param point The point to check.
 * @return True if the point is inside, false otherwise.
 */
bool RigidBox::IsPointInside(const glm::dvec3& point) const {
  glm::dvec3 local_point = glm::inverse(rotation_) * (point - position_);
  return glm::all(glm::lessThanEqual(local_point, dimensions_ / 2.0)) &&
         glm::all(glm::greaterThanEqual(local_point, -dimensions_ / 2.0));
}

/**
 * @brief Calculates the inertia tensor of the box.
 * @param mass The mass of the box.
 * @param dimensions The dimensions of the box.
 * @return The inertia tensor of the box.
 */
glm::dmat3 RigidBox::CalculateBoxInertia(double mass,
                                         const glm::dvec3& dimensions) {
  double Ixx =
      mass * (dimensions.y * dimensions.y + dimensions.z * dimensions.z) / 12.0;
  double Iyy =
      mass * (dimensions.x * dimensions.x + dimensions.z * dimensions.z) / 12.0;
  double Izz =
      mass * (dimensions.x * dimensions.x + dimensions.y * dimensions.y) / 12.0;

  return {Ixx, 0.0, 0.0, 0.0, Iyy, 0.0, 0.0, 0.0, Izz};
}

/**
 * @brief Draws the rigid box using OpenGL.
 */
void RigidBox::Draw() const {
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glTranslated(position_.x, position_.y, position_.z);
  glMultMatrixd(glm::value_ptr(glm::mat4_cast(rotation_)));

  const glm::dvec3 halfSize = dimensions_ / 2.0;

  glBegin(GL_QUADS);
  // Front face (Z+)
  glColor3fv(glm::value_ptr(face_colors[0]));
  glVertex3d(-halfSize.x, -halfSize.y, halfSize.z);
  glVertex3d(halfSize.x, -halfSize.y, halfSize.z);
  glVertex3d(halfSize.x, halfSize.y, halfSize.z);
  glVertex3d(-halfSize.x, halfSize.y, halfSize.z);

  // Back face (Z-)
  glColor3fv(glm::value_ptr(face_colors[1]));
  glVertex3d(-halfSize.x, -halfSize.y, -halfSize.z);
  glVertex3d(-halfSize.x, halfSize.y, -halfSize.z);
  glVertex3d(halfSize.x, halfSize.y, -halfSize.z);
  glVertex3d(halfSize.x, -halfSize.y, -halfSize.z);

  // Top face (Y+)
  glColor3fv(glm::value_ptr(face_colors[2]));
  glVertex3d(-halfSize.x, halfSize.y, -halfSize.z);
  glVertex3d(-halfSize.x, halfSize.y, halfSize.z);
  glVertex3d(halfSize.x, halfSize.y, halfSize.z);
  glVertex3d(halfSize.x, halfSize.y, -halfSize.z);

  // Bottom face (Y-)
  glColor3fv(glm::value_ptr(face_colors[3]));
  glVertex3d(-halfSize.x, -halfSize.y, -halfSize.z);
  glVertex3d(halfSize.x, -halfSize.y, -halfSize.z);
  glVertex3d(halfSize.x, -halfSize.y, halfSize.z);
  glVertex3d(-halfSize.x, -halfSize.y, halfSize.z);

  // Right face (X+)
  glColor3fv(glm::value_ptr(face_colors[4]));
  glVertex3d(halfSize.x, -halfSize.y, -halfSize.z);
  glVertex3d(halfSize.x, halfSize.y, -halfSize.z);
  glVertex3d(halfSize.x, halfSize.y, halfSize.z);
  glVertex3d(halfSize.x, -halfSize.y, halfSize.z);

  // Left face (X-)
  glColor3fv(glm::value_ptr(face_colors[5]));
  glVertex3d(-halfSize.x, -halfSize.y, -halfSize.z);
  glVertex3d(-halfSize.x, -halfSize.y, halfSize.z);
  glVertex3d(-halfSize.x, halfSize.y, halfSize.z);
  glVertex3d(-halfSize.x, halfSize.y, -halfSize.z);
  glEnd();

  glPopMatrix();
}

}  // namespace rigid_dynamics::bodies