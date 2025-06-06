
#ifndef RIGIDDYNAMICS_BODIES_RIGID_BODY_H_
#define RIGIDDYNAMICS_BODIES_RIGID_BODY_H_

#include <glm/ext.hpp>
#include <glm/glm.hpp>
#include <memory>
#include <utility>
#include <vector>

namespace rigid_dynamics::bodies {

class RigidBox;
class RigidSphere;

namespace contacts {
struct Contact;
}

class RigidBody {
 public:
  enum class ShapeType { BOX, SPHERE };

 protected:
  // Constant quantities
  double mass_;
  glm::dmat3 body_inertia_;
  double restitution_;

  // State variables
  glm::dvec3 position_;
  glm::dquat rotation_;
  glm::dvec3 momentum_;
  glm::dvec3 angular_momentum_;
  glm::dmat3 inertia_{body_inertia_};
  bool fixed_{false};

  // Derived quantities
  glm::dvec3 applied_force_{};
  glm::dvec3 applied_torque_{};

  struct Derivatives {
    glm::dvec3 velocity;
    glm::dquat spin;
    glm::dvec3 force;
    glm::dvec3 torque;

    Derivatives& operator*=(double scalar);
    Derivatives& operator+=(const Derivatives& other);

    Derivatives operator*(double scalar) const;
    Derivatives operator+(const Derivatives& other) const;
  };

 public:
  RigidBody(double mass, const glm::dmat3& body_inertia, double restitution,
            const glm::dvec3& position = {0.0, 0.0, 0.0},
            const glm::dquat& rotation = {1.0, 0.0, 0.0, 0.0},
            const glm::dvec3& momentum = {0.0, 0.0, 0.0},
            const glm::dvec3& angular_momentum = {0.0, 0.0, 0.0});

  explicit RigidBody(double restitution,
                     const glm::dvec3& position = {0.0, 0.0, 0.0},
                     const glm::dquat& rotation = {1.0, 0.0, 0.0, 0.0});
  virtual ~RigidBody() = default;
  [[nodiscard]] virtual std::unique_ptr<RigidBody> Clone() const = 0;

  [[nodiscard]] virtual Derivatives ComputeDerivatives(double time) const;
  void ApplyDerivatives(const Derivatives& derivatives, double dt);
  void IntegrateRungeKutta(double time, double h);

  void UpdateInertia();

  [[nodiscard]] virtual ShapeType GetShapeType() const = 0;

  [[nodiscard]] glm::dvec3 GetPointVelocity(const glm::dvec3& point) const;

  void AddMomentum(const glm::dvec3& momentum);
  void AddAngularMomentum(const glm::dvec3& angular_momentum);
  void AddPosition(const glm::dvec3& position);
  void ApplyForce(const glm::dvec3& force);
  void ApplyTorque(const glm::dvec3& torque);

  [[nodiscard]] double GetMass() const;
  [[nodiscard]] const glm::dmat3& GetBodyInertia() const;
  [[nodiscard]] double GetRestitution() const;
  [[nodiscard]] const glm::dvec3& GetPosition() const;
  [[nodiscard]] const glm::dquat& GetRotation() const;
  [[nodiscard]] const glm::dvec3& GetMomentum() const;
  [[nodiscard]] const glm::dvec3& GetAngularMomentum() const;
  [[nodiscard]] const glm::dmat3& GetInertia() const;
  [[nodiscard]] const glm::dvec3& GetAppliedForce() const;
  [[nodiscard]] const glm::dvec3& GetAppliedTorque() const;
  [[nodiscard]] bool IsFixed() const;

  [[nodiscard]] glm::dvec3 GetLinearVelocity() const;
  [[nodiscard]] glm::dvec3 GetAngularVelocity() const;

  [[nodiscard]] double CalculateLinearKineticEnergy() const;
  [[nodiscard]] double CalculateAngularKineticEnergy() const;
  [[nodiscard]] double CalculatePotentialEnergy(
      const glm::dvec3& gravity, double reference_height = 0.0) const;
  [[nodiscard]] double CalculateTotalEnergy(
      const glm::dvec3& gravity, double reference_height = 0.0) const;
};

}  // namespace rigid_dynamics::bodies

#endif  // RIGIDDYNAMICS_BODIES_RIGID_BODY_H_