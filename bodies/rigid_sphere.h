
#ifndef RIGIDDYNAMICS_BODIES_RIGID_SPHERE_H_
#define RIGIDDYNAMICS_BODIES_RIGID_SPHERE_H_

#include "drawable_rigid_body.h"
#include "random_color_generator.h"

#define SLICES 100
#define STACKS 100

namespace rigid_dynamics::bodies {

class RigidSphere : public DrawableRigidBody {
 public:
  RigidSphere(
      double mass, double radius, double restitution,
      const glm::dvec3& position = {0.0, 0.0, 0.0},
      const glm::dquat& rotation = {1.0, 0.0, 0.0, 0.0},
      const glm::dvec3& momentum = {0.0, 0.0, 0.0},
      const glm::dvec3& angular_momentum = {0.0, 0.0, 0.0},
      const glm::vec3& color = random_color_generator_.GenerateRandomColor());

  [[nodiscard]] std::unique_ptr<RigidBody> Clone() const override;
  [[nodiscard]] ShapeType GetShapeType() const override;
  [[nodiscard]] double GetRadius() const;

  void Draw() const override;

 private:
  double radius_;
  glm::vec3 color_;
  inline static RandomColorGenerator random_color_generator_{};

  [[nodiscard]] static glm::dmat3 CalculateSphereInertia(double mass,
                                                         double radius);
};

}  // namespace rigid_dynamics::bodies

#endif  // RIGIDDYNAMICS_BODIES_RIGID_SPHERE_H_
