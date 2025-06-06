/**
 * @file rigid_body.cc
 * @brief Implementation of the RigidBody class.
 */

#include "rigid_body.h"

#include <iostream>

#include "contacts/contact.h"
#include "rigid_box.h"
#include "rigid_sphere.h"

namespace rigid_dynamics::bodies {

/**
 * @brief Constructor for RigidBody.
 * @param mass The mass of the rigid body.
 * @param body_inertia The inertia tensor of the rigid body in its body frame.
 * @param restitution The restitution coefficient of the rigid body.
 * @param position The position of the rigid body.
 * @param rotation The rotation of the rigid body as a quaternion.
 * @param momentum The linear momentum of the rigid body.
 * @param angular_momentum The angular momentum of the rigid body.
 */
RigidBody::RigidBody(double mass, const glm::dmat3& body_inertia,
                     double restitution, const glm::dvec3& position,
                     const glm::dquat& rotation, const glm::dvec3& momentum,
                     const glm::dvec3& angular_momentum)
    : mass_(mass),
      body_inertia_(body_inertia),
      restitution_(restitution),
      position_(position),
      rotation_(glm::normalize(rotation)),
      momentum_(momentum),
      angular_momentum_(angular_momentum) {}

RigidBody::RigidBody(double restitution, const glm::dvec3& position,
                     const glm::dquat& rotation)
    : mass_(std::numeric_limits<double>::infinity()),
      body_inertia_(glm::dmat3(std::numeric_limits<double>::infinity())),
      restitution_(restitution),
      position_(position),
      rotation_(glm::normalize(rotation)),
      momentum_({0.0, 0.0, 0.0}),
      angular_momentum_({0.0, 0.0, 0.0}),
      fixed_(true) {}

/**
 * @brief Computes the derivatives of the rigid body's state.
 * @param time The current time.
 * @return The derivatives of the rigid body's state.
 */
RigidBody::Derivatives RigidBody::ComputeDerivatives(double time) const {
  return Derivatives{GetLinearVelocity(),
                     0.5 * glm::dquat(0.0, GetAngularVelocity()) * rotation_,
                     applied_force_, applied_torque_};
}

/**
 * @brief Applies the derivatives to the rigid body's state.
 * @param derivatives The derivatives of the rigid body's state.
 * @param dt The time step.
 */
void RigidBody::ApplyDerivatives(const Derivatives& derivatives, double dt) {
  position_ += derivatives.velocity * dt;
  rotation_ = glm::normalize(rotation_ + derivatives.spin * dt);
  momentum_ += derivatives.force * dt;
  angular_momentum_ += derivatives.torque * dt;

  UpdateInertia();
}

/**
 * @brief Integrates the rigid body's state using the Runge-Kutta method.
 * @param h The time step.
 * @param time The current time.
 */
void RigidBody::IntegrateRungeKutta(double time, double h) {
  if (fixed_) return;

  const auto kComputeStep = [this, time](const Derivatives& derivatives,
                                         double step_scale,
                                         double time_offset) {
    auto temp = Clone();
    temp->applied_force_ = applied_force_;
    temp->applied_torque_ = applied_torque_;

    temp->ApplyDerivatives(derivatives, step_scale);
    return temp->ComputeDerivatives(time + time_offset);
  };

  const Derivatives k1 = ComputeDerivatives(time);
  const Derivatives k2 = kComputeStep(k1, h / 2, h / 2);
  const Derivatives k3 = kComputeStep(k2, h / 2, h / 2);
  const Derivatives k4 = kComputeStep(k3, h, h);

  ApplyDerivatives(k1 + k2 * 2 + k3 * 2 + k4, h / 6.0);

  // Clear aaplied forces and torques
  applied_force_ = glm::dvec3();
  applied_torque_ = glm::dvec3();
}

/**
 * @brief Updates the inertia tensor of the rigid body.
 */
void RigidBody::UpdateInertia() {
  glm::dmat3 matrix = glm::mat3_cast(rotation_);
  inertia_ = matrix * body_inertia_ * glm::transpose(matrix);
}

glm::dvec3 RigidBody::GetPointVelocity(const glm::dvec3& point) const {
  if (fixed_) return glm::dvec3(0.0);
  return GetLinearVelocity() + cross(GetAngularVelocity(), point - position_);
}

void RigidBody::AddMomentum(const glm::dvec3& momentum) {
  momentum_ += momentum;
}

void RigidBody::AddAngularMomentum(const glm::dvec3& angular_momentum) {
  angular_momentum_ += angular_momentum;
}

void RigidBody::AddPosition(const glm::dvec3& position) {
  position_ += position;
}

void RigidBody::ApplyForce(const glm::dvec3& force) { applied_force_ += force; }

void RigidBody::ApplyTorque(const glm::dvec3& torque) {
  applied_torque_ += torque;
}

/**
 * @brief Gets the mass of the rigid body.
 * @return The mass of the rigid body.
 */
double RigidBody::GetMass() const { return mass_; }

/**
 * @brief Gets the inertia tensor of the rigid body in its body frame.
 * @return The inertia tensor of the rigid body in its body frame.
 */
const glm::dmat3& RigidBody::GetBodyInertia() const { return body_inertia_; }

/**
 * @brief Gets the restitution coefficient of the rigid body.
 * @return The restitution coefficient of the rigid body.
 */
double RigidBody::GetRestitution() const { return restitution_; }

/**
 * @brief Gets the position of the rigid body.
 * @return The position of the rigid body.
 */
const glm::dvec3& RigidBody::GetPosition() const { return position_; }

/**
 * @brief Gets the rotation of the rigid body as a quaternion.
 * @return The rotation of the rigid body as a quaternion.
 */
const glm::dquat& RigidBody::GetRotation() const { return rotation_; }

/**
 * @brief Gets the linear momentum of the rigid body.
 * @return The linear momentum of the rigid body.
 */
const glm::dvec3& RigidBody::GetMomentum() const { return momentum_; }

/**
 * @brief Gets the angular momentum of the rigid body.
 * @return The angular momentum of the rigid body.
 */
const glm::dvec3& RigidBody::GetAngularMomentum() const {
  return angular_momentum_;
}

/**
 * @brief Gets the linear velocity of the rigid body.
 * @return The linear velocity of the rigid body.
 */
glm::dvec3 RigidBody::GetLinearVelocity() const { return momentum_ / mass_; }

/**
 * @brief Gets the angular velocity of the rigid body.
 * @return The angular velocity of the rigid body.
 */
glm::dvec3 RigidBody::GetAngularVelocity() const {
  return angular_momentum_ / inertia_;
}

/**
 * @brief Gets the inertia tensor of the rigid body.
 * @return The inertia tensor of the rigid body.
 */
const glm::dmat3& RigidBody::GetInertia() const { return inertia_; }

const glm::dvec3& RigidBody::GetAppliedForce() const { return applied_force_; }
const glm::dvec3& RigidBody::GetAppliedTorque() const {
  return applied_torque_;
}

bool RigidBody::IsFixed() const { return fixed_; }

/**
 * @brief Calculates the linear kinetic energy of the rigid body.
 * @return The linear kinetic energy.
 */
double RigidBody::CalculateLinearKineticEnergy() const {
  return 0.5 * glm::dot(momentum_, momentum_) / mass_;
}

/**
 * @brief Calculates the angular kinetic energy of the rigid body.
 * @return The angular kinetic energy.
 */
double RigidBody::CalculateAngularKineticEnergy() const {
  return 0.5 * glm::dot(angular_momentum_, GetAngularVelocity());
}

/**
 * @brief Calculates the potential energy of the rigid body.
 * @param gravity The gravitational acceleration vector.
 * @param reference_height The reference height for potential energy
 * calculation.
 * @return The potential energy.
 */
double RigidBody::CalculatePotentialEnergy(const glm::dvec3& gravity,
                                           double reference_height) const {
  return -mass_ * glm::dot(gravity, position_) + mass_ * reference_height;
}

/**
 * @brief Calculates the total energy of the rigid body.
 * @return The total energy.
 */
double RigidBody::CalculateTotalEnergy(const glm::dvec3& gravity,
                                       double reference_height) const {
  if (fixed_) return 0.0;
  return CalculateLinearKineticEnergy() + CalculateAngularKineticEnergy() +
         CalculatePotentialEnergy(gravity, reference_height);
}

/**
 * @brief Overloads the *= operator for the Derivatives struct.
 * @param scalar The scalar value to multiply by.
 * @return The Derivatives struct multiplied by the scalar value.
 */
RigidBody::Derivatives& RigidBody::Derivatives::operator*=(double scalar) {
  velocity *= scalar;
  spin *= scalar;
  force *= scalar;
  torque *= scalar;
  return *this;
}

/**
 * @brief Overloads the += operator for the Derivatives struct.
 * @param other The other Derivatives struct to add to.
 * @return The Derivatives struct with the added values.
 */
RigidBody::Derivatives& RigidBody::Derivatives::operator+=(
    const RigidBody::Derivatives& other) {
  velocity += other.velocity;
  spin += other.spin;
  force += other.force;
  torque += other.torque;
  return *this;
}

/**
 * @brief Overloads the * operator for the Derivatives struct.
 * @param scalar The scalar value to multiply by.
 * @return The Derivatives struct multiplied by the scalar value.
 */
RigidBody::Derivatives RigidBody::Derivatives::operator*(double scalar) const {
  return Derivatives(*this) *= scalar;
}

/**
 * @brief Overloads the + operator for the Derivatives struct.
 * @param other The other Derivatives struct to add to.
 * @return The Derivatives struct with the added values.
 */
RigidBody::Derivatives RigidBody::Derivatives::operator+(
    const RigidBody::Derivatives& other) const {
  return Derivatives(*this) += other;
}

}  // namespace rigid_dynamics::bodies