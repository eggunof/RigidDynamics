#include "camera.h"

#include <glm/ext.hpp>
#include <iomanip>
#include <sstream>

namespace rigid_dynamics {

Camera::Camera(glm::vec3 position, glm::vec3 world_up, double yaw, double pitch)
    : world_up_(world_up), position_(position), yaw_(yaw), pitch_(pitch) {
  UpdateCameraVectors();
}

glm::mat4 Camera::GetViewMatrix() const {
  return lookAt(position_, position_ + front_, up_);
}

void Camera::UpdatePosition(double deltaTime) {
  velocity_ += acceleration_ * deltaTime;

  if (glm::length(acceleration_) == 0.0) {
    if (glm::length(velocity_) > 0.0) {
      velocity_ -= glm::normalize(velocity_) * deceleration_rate_ * deltaTime;

      if (glm::length(velocity_) < 1.0) {
        velocity_ = glm::dvec3(0.0);
      }
    }
  }

  if (glm::length(velocity_) > max_speed_) {
    velocity_ = glm::normalize(velocity_) * max_speed_;
  }

  position_ += velocity_ * deltaTime;
}

void Camera::ProcessKeyboard(
    const std::vector<MovementDirections> &directions) {
  glm::dvec3 directions_sum;
  for (auto &direction : directions) {
    switch (direction) {
      case MovementDirections::FORWARD:
        directions_sum += move_front_;
        break;
      case MovementDirections::BACKWARD:
        directions_sum -= move_front_;
        break;
      case MovementDirections::LEFT:
        directions_sum -= move_right_;
        break;
      case MovementDirections::RIGHT:
        directions_sum += move_right_;
        break;
      case MovementDirections::UP:
        directions_sum += world_up_;
        break;
      case MovementDirections::DOWN:
        directions_sum -= world_up_;
        break;
    }
  }

  if (glm::length(directions_sum) > 0.0) {
    acceleration_ = glm::normalize(directions_sum) * acceleration_rate_;
  } else {
    acceleration_ = glm::dvec3(0.0);
  }
}

void Camera::ProcessMouseMovement(double xoffset, double yoffset) {
  xoffset *= mouse_sensitivity_;
  yoffset *= mouse_sensitivity_;

  yaw_ += xoffset;
  pitch_ += yoffset;

  if (pitch_ > 89.0f) pitch_ = 89.0f;
  if (pitch_ < -89.0f) pitch_ = -89.0f;

  if (yaw_ > 180.0f) yaw_ = -180.0f;
  if (yaw_ < -180.0f) yaw_ = 180.0f;

  UpdateCameraVectors();
}

void Camera::ProcessMouseScroll(double yoffset) {
  zoom_ -= yoffset;
  if (zoom_ < 1.0f) zoom_ = 1.0f;
  if (zoom_ > 45.0f) zoom_ = 45.0f;
}

void Camera::UpdateCameraVectors() {
  front_ = normalize(
      glm::dvec3(cos(glm::radians(yaw_)) * cos(glm::radians(pitch_)),
                 sin(glm::radians(pitch_)),
                 sin(glm::radians(yaw_)) * cos(glm::radians(pitch_))));

  move_front_ = normalize(
      glm::dvec3(cos(glm::radians(yaw_)), 0, sin(glm::radians(yaw_))));

  move_right_ = normalize(cross(move_front_, world_up_));
  up_ = normalize(cross(move_right_, front_));
}

const glm::dvec3 &Camera::GetPosition() const { return position_; }
const glm::dvec3 &Camera::GetFront() const { return front_; }
const glm::dvec3 &Camera::GetUp() const { return up_; }
double Camera::GetZoom() const { return zoom_; }

std::string Camera::GetPositionString() const {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2);
  oss << "Position: (" << position_.x << ", " << position_.y << ", "
      << position_.z << ")";
  return oss.str();
}

std::string Camera::GetVelocityString() const {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2);
  oss << "Velocity: (" << velocity_.x << ", " << velocity_.y << ", "
      << velocity_.z << ")";
  return oss.str();
}

std::string Camera::GetFacingString() const {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2);
  oss << "Facing: (" << front_.x << ", " << front_.y << ", " << front_.z << ")";
  oss << "Yaw: " << yaw_ << " Pitch: " << pitch_;
  return oss.str();
}

glm::dquat Camera::GetOrientation() const {
  return {glm::dvec3(glm::radians(pitch_), glm::radians(-yaw_ - 90.0), 0.0)};
}

}  // namespace rigid_dynamics