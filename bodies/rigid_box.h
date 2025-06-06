
#ifndef RIGIDDYNAMICS_BODIES_RIGID_BOX_H_
#define RIGIDDYNAMICS_BODIES_RIGID_BOX_H_

#include <array>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "drawable_rigid_body.h"
#include "random_color_generator.h"

namespace rigid_dynamics::bodies {

class RigidBox : public DrawableRigidBody {
 public:
  RigidBox(const glm::dvec3& dimensions, double mass, double restitution,
           const glm::dvec3& position = {0.0, 0.0, 0.0},
           const glm::dquat& rotation = {1.0, 0.0, 0.0, 0.0},
           const glm::dvec3& momentum = {0.0, 0.0, 0.0},
           const glm::dvec3& angular_momentum = {0.0, 0.0, 0.0},
           const std::array<glm::vec3, 6>& face_colors =
               random_color_generator_.GenerateRandomColors<6>());

  RigidBox(const glm::dvec3& dimensions, double restitution,
           const glm::dvec3& position = {0.0, 0.0, 0.0},
           const glm::dquat& rotation = {1.0, 0.0, 0.0, 0.0},
           const std::array<glm::vec3, 6>& face_colors =
               random_color_generator_.GenerateRandomColors<6>());

  [[nodiscard]] std::unique_ptr<RigidBody> Clone() const override;
  [[nodiscard]] ShapeType GetShapeType() const override;
  [[nodiscard]] const glm::dvec3& GetDimensions() const;
  [[nodiscard]] std::array<glm::dvec3, 8> GetVertices() const;
  [[nodiscard]] std::array<glm::dvec3, 3> GetAxes() const;

  [[nodiscard]] glm::dvec3 GetClosestPoint(const glm::dvec3& point) const;
  [[nodiscard]] bool IsPointInside(const glm::dvec3& point) const;

  void Draw() const override;

 private:
  glm::dvec3 dimensions_;
  std::array<glm::vec3, 6> face_colors{};
  inline static RandomColorGenerator random_color_generator_{};

  [[nodiscard]] static glm::dmat3 CalculateBoxInertia(
      double mass, const glm::dvec3& dimensions);
};

}  // namespace rigid_dynamics::bodies

#endif  // RIGIDDYNAMICS_BODIES_RIGID_BOX_H_
