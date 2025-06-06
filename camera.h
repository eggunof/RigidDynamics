
#ifndef RIGIDDYNAMICS_CAMERA_H_
#define RIGIDDYNAMICS_CAMERA_H_

#include <glm/glm.hpp>
#include <string>
#include <vector>

namespace rigid_dynamics {

class Camera {
 private:
  constexpr static double YAW = -90.0;
  constexpr static double PITCH = 0.0;
  constexpr static double MAX_SPEED = 100.0;
  constexpr static double ACCELERATION_RATE = 150.0;
  constexpr static double DECELERATION_RATE = 100.0;
  constexpr static double MOUSE_SENSITIVITY = 0.05;
  constexpr static double ZOOM = 45.0;

 public:
  enum class MovementDirections { FORWARD, BACKWARD, LEFT, RIGHT, UP, DOWN };

  explicit Camera(glm::vec3 position = {0.0f, 0.0f, 0.0f},
                  glm::vec3 world_up = {0.0f, 1.0f, 0.0f}, double yaw = YAW,
                  double pitch = PITCH);

  void UpdatePosition(double deltaTime);

  [[nodiscard]] glm::mat4 GetViewMatrix() const;

  void ProcessKeyboard(const std::vector<MovementDirections> &directions);
  void ProcessMouseMovement(double xoffset, double yoffset);
  void ProcessMouseScroll(double yoffset);

  [[nodiscard]] const glm::dvec3 &GetPosition() const;
  [[nodiscard]] const glm::dvec3 &GetFront() const;
  [[nodiscard]] const glm::dvec3 &GetUp() const;
  [[nodiscard]] double GetZoom() const;
  [[nodiscard]] glm::dquat GetOrientation() const;

  [[nodiscard]] std::string GetPositionString() const;
  [[nodiscard]] std::string GetVelocityString() const;
  [[nodiscard]] std::string GetFacingString() const;

 private:
  glm::dvec3 world_up_;

  glm::dvec3 position_;
  glm::dvec3 velocity_{};
  glm::dvec3 acceleration_{};
  glm::dvec3 front_{};
  glm::dvec3 up_{};

  glm::dvec3 move_front_{};
  glm::dvec3 move_right_{};

  double yaw_{YAW};
  double pitch_{PITCH};

  double max_speed_{MAX_SPEED};
  double acceleration_rate_{ACCELERATION_RATE};
  double deceleration_rate_{DECELERATION_RATE};

  double mouse_sensitivity_{MOUSE_SENSITIVITY};
  double zoom_{ZOOM};

 private:
  void UpdateCameraVectors();
};

}  // namespace rigid_dynamics

#endif  // RIGIDDYNAMICS_CAMERA_H_
