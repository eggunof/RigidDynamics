
#include "body_manager.h"

namespace rigid_dynamics::bodies {

void BodyManager::AddBody(const std::shared_ptr<DrawableRigidBody> &body) {
  for (auto &other_body : bodies_) {
    if (body->IsFixed() && other_body->IsFixed()) continue;

    interactions_.push_back(
        std::make_shared<contacts::BodyInteraction>(other_body, body));
  }
  bodies_.push_back(body);
}

void BodyManager::ClearBodies() {
  bodies_.clear();
  interactions_.clear();
}

void BodyManager::ApplyGravity() const {
  for (auto &body : bodies_)
    body->ApplyForce(gravity_acceleration_ * body->GetMass());
}

void BodyManager::ProcessInteractions() {
  static constexpr double collision_threshold = 0.3;

  for (auto &interaction : interactions_) {
    interaction->ProcessContacts(collision_threshold);
  }
}

void BodyManager::IntegrateAll(double time, double h) const {
  for (auto &body : bodies_) {
    body->IntegrateRungeKutta(time, h);
  }
}

void BodyManager::DrawAll() const {
  for (auto &body : bodies_) {
    body->Draw();
  }
}

size_t BodyManager::GetBodyCount() const { return bodies_.size(); }
std::shared_ptr<DrawableRigidBody> BodyManager::GetBody(size_t index) const {
  return bodies_[index];
}

void BodyManager::RemoveBody(size_t i) {
  if (i >= bodies_.size()) return;
  bodies_.erase(bodies_.begin() + i);
  for (size_t j = 0; j < interactions_.size(); ++j) {
    if (interactions_[j]->body1 == bodies_[i] ||
        interactions_[j]->body2 == bodies_[i]) {
      interactions_.erase(interactions_.begin() + j);
    }
  }
}

const glm::dvec3 &BodyManager::GetGravityAcceleration() const {
  return gravity_acceleration_;
}

void BodyManager::SetGravityAcceleration(
    const glm::dvec3 &gravity_acceleration) {
  gravity_acceleration_ = gravity_acceleration;
}

}  // namespace rigid_dynamics::bodies