
#include "render.h"

#include <iomanip>
#include <iostream>
#include <memory>
#include <random>

#include "bodies/body_manager.h"
#include "bodies/drawable_rigid_body.h"
#include "bodies/rigid_box.h"
#include "bodies/rigid_sphere.h"
#include "camera.h"

namespace rigid_dynamics {
bool keys[256];
bool isGravityEnabled = true;

double deltaTime = 0.0;
int lastFrameTime = 0;
int frameCount = 0;
int lastFpsUpdate = 0;
double fps = 0.0;

bodies::BodyManager body_manager;

std::unique_ptr<Camera> camera =
    std::make_unique<Camera>(glm::dvec3(0.0, 100.0, 100.0));

glm::dvec3 gravity_acceleration = {0.0, -GRAVITATIONAL_ACCELERATION, 0.0};

void DrawText(const glm::dvec2& position, const std::string& text,
              const glm::dvec3& color, void* font) {
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0, glutGet(GLUT_WINDOW_WIDTH), 0, glutGet(GLUT_WINDOW_HEIGHT));

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  glDisable(GL_DEPTH_TEST);
  glColor3dv(glm::value_ptr(color));

  glRasterPos2dv(glm::value_ptr(position));
  for (char c : text) {
    glutBitmapCharacter(font, c);
  }

  glEnable(GL_DEPTH_TEST);
  glPopMatrix();

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
}

void Display() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  auto position = camera->GetPosition();
  auto front = camera->GetFront();
  auto up = camera->GetUp();
  auto center = position + front;

  gluLookAt(position.x, position.y, position.z, center.x, center.y, center.z,
            up.x, up.y, up.z);

  body_manager.DrawAll();

  double system_total_energy = 0.0;
  for (size_t i = 0; i < body_manager.GetBodyCount(); ++i) {
    auto body = body_manager.GetBody(i);
    std::stringstream ss;
    ss << "Body " << i + 1 << ": ";
    double body_total_energy = body->CalculateTotalEnergy(
        isGravityEnabled ? gravity_acceleration : glm::dvec3(0.0), 0.0);
    system_total_energy += body_total_energy;
    ss << " Total Energy: " << body_total_energy;
    DrawText({10, HEIGHT - (180 + 30 * (i + 1))}, ss.str());
  }

  std::stringstream ss;
  ss << "System Total Energy: " << system_total_energy;
  DrawText({10, HEIGHT - 180}, ss.str(), {1.0, 0.5, 0.0},
           GLUT_BITMAP_TIMES_ROMAN_24);

  DrawText({10, HEIGHT - 30}, "FPS: " + std::to_string(static_cast<int>(fps)));
  DrawText({10, HEIGHT - 60}, camera->GetPositionString());
  DrawText({10, HEIGHT - 90}, camera->GetVelocityString());
  DrawText({10, HEIGHT - 120}, camera->GetFacingString());
  if (isGravityEnabled) {
    DrawText({1750, HEIGHT - 30}, "Gravity Enabled", {0.0, 1.0, 0.0});
  } else {
    DrawText({1750, HEIGHT - 30}, "Gravity Disabled", {1.0, 0.0, 0.0});
  }

  glutSwapBuffers();
}

void Reshape(int width, int height) {
  WIDTH = width;
  HEIGHT = height;

  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(camera->GetZoom(), static_cast<double>(width) / height, 0.1,
                 1000.0);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void ToggleFullScreen() {
  static bool isFullScreen = false;
  isFullScreen = !isFullScreen;

  if (isFullScreen) {
    int screenWidth = glutGet(GLUT_SCREEN_WIDTH);
    int screenHeight = glutGet(GLUT_SCREEN_HEIGHT);

    glutFullScreen();

    WIDTH = screenWidth;
    HEIGHT = screenHeight;
  } else {
    glutReshapeWindow(800, 600);
    glutPositionWindow(100, 100);

    WIDTH = 800;
    HEIGHT = 600;
  }
}

void Idle() {
  int currentFrameTime = glutGet(GLUT_ELAPSED_TIME);
  deltaTime = static_cast<float>(currentFrameTime - lastFrameTime) / 1000.0;
  lastFrameTime = currentFrameTime;

  frameCount++;

  if (currentFrameTime - lastFpsUpdate >= 1000) {
    fps = frameCount * 1000.0 / (currentFrameTime - lastFpsUpdate);
    frameCount = 0;
    lastFpsUpdate = currentFrameTime;
  }

  std::vector<Camera::MovementDirections> directions;
  if (keys['w']) directions.push_back(Camera::MovementDirections::FORWARD);
  if (keys['s']) directions.push_back(Camera::MovementDirections::BACKWARD);
  if (keys['a']) directions.push_back(Camera::MovementDirections::LEFT);
  if (keys['d']) directions.push_back(Camera::MovementDirections::RIGHT);
  if (keys[' ']) directions.push_back(Camera::MovementDirections::UP);
  if (keys['c']) directions.push_back(Camera::MovementDirections::DOWN);
  camera->ProcessKeyboard(directions);
  camera->UpdatePosition(deltaTime);

  for (int i = 0; i < body_manager.GetBodyCount(); ++i) {
    auto body = body_manager.GetBody(i);
    if (body->GetPosition().y < -1000.0) {
      body_manager.RemoveBody(i);
    }
  }

  if (isGravityEnabled) body_manager.ApplyGravity();
  body_manager.ProcessInteractions();
  body_manager.IntegrateAll(lastFrameTime, deltaTime);

  glutPostRedisplay();
}

void Keyboard(unsigned char key, int x, int y) {
  key = tolower(key);
  if (key == 27) {
    glutDestroyWindow(glutGetWindow());
    exit(0);
  }
  if (key == 'f') {
    ToggleFullScreen();
  }
  if (key == 'g') {
    isGravityEnabled = !isGravityEnabled;
  }
  keys[key] = true;
}

void KeyboardUp(unsigned char key, int x, int y) { keys[tolower(key)] = false; }

void PassiveMotion(int x, int y) {
  static int centerX = WIDTH / 2;
  static int centerY = HEIGHT / 2;
  centerX = WIDTH / 2;
  centerY = HEIGHT / 2;

  int dx = x - centerX;
  int dy = y - centerY;

  if (abs(dx) < 1 && abs(dy) < 1) return;

  camera->ProcessMouseMovement(dx, -dy);

  glutWarpPointer(WIDTH / 2, HEIGHT / 2);
}

void Mouse(int button, int state, int x, int y) {
  if ((button == 3 || button == 4) && state != GLUT_UP) {
    camera->ProcessMouseScroll(button == 3 ? 1 : -1);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(camera->GetZoom(), static_cast<double>(WIDTH) / HEIGHT, 0.1,
                   1000.0);
  }
  if ((button == GLUT_LEFT_BUTTON || button == GLUT_RIGHT_BUTTON) &&
      state == GLUT_DOWN) {
    glm::dvec3 spawn_position =
        camera->GetPosition() + 10.0 * camera->GetFront();
    glm::dquat spawn_rotation = camera->GetOrientation();

    static std::mt19937 rng(std::random_device{}());
    static std::uniform_real_distribution<double> dist(0.5, 5.0);

    if (button == GLUT_LEFT_BUTTON) {
      auto box = std::make_shared<bodies::RigidBox>(
          glm::dvec3{dist(rng), dist(rng), dist(rng)}, dist(rng), 1.0,
          spawn_position, spawn_rotation);
      body_manager.AddBody(box);
    } else {
      auto sphere = std::make_shared<bodies::RigidSphere>(
          dist(rng), dist(rng), 1.0, spawn_position, spawn_rotation);
      body_manager.AddBody(sphere);
    }
  }
}

void Run(int argc, char** argv) {
  glutInit(&argc, argv);

  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(WIDTH, HEIGHT);
  glutInitWindowPosition(0, 0);
  glutCreateWindow("Rigid Dynamics");

  glClearColor(0.3, 0.5, 0.7, 0);

  glutReshapeFunc(Reshape);
  glutDisplayFunc(Display);
  glutIdleFunc(Idle);
  glutKeyboardFunc(Keyboard);
  glutKeyboardUpFunc(KeyboardUp);
  glutPassiveMotionFunc(PassiveMotion);
  glutMouseFunc(Mouse);

  glEnable(GL_DEPTH_TEST);

  glutSetCursor(GLUT_CURSOR_NONE);

  body_manager.SetGravityAcceleration(gravity_acceleration);

  body_manager.AddBody(std::make_shared<bodies::RigidBox>(
      glm::dvec3{500.0, 2.0, 500.0}, 1.0, glm::dvec3{-100.0, 0.0, 0.0},
      glm::dquat{{0.0, 0.0, glm::radians(20.0)}},
      std::array{glm::vec3{1.0, 0.0, 0.0}, glm::vec3{1.0, 0.0, 0.0},
                 glm::vec3{0.7, 0.7, 0.7}, glm::vec3{0.7, 0.7, 0.7},
                 glm::vec3{0.0, 1.0, 0.0}, glm::vec3{0.0, 1.0, 0.0}}));
  body_manager.AddBody(std::make_shared<bodies::RigidBox>(
      glm::dvec3{500.0, 2.0, 500.0}, 1.0, glm::dvec3{100.0, 0.0, 0.0},
      glm::dquat{{0.0, 0.0, glm::radians(-20.0)}},
      std::array{glm::vec3{1.0, 0.0, 0.0}, glm::vec3{1.0, 0.0, 0.0},
                 glm::vec3{0.3, 0.3, 0.3}, glm::vec3{0.3, 0.3, 0.3},
                 glm::vec3{0.0, 1.0, 0.0}, glm::vec3{0.0, 1.0, 0.0}}));
  /* Horizontal
  body_manager.AddBody(std::make_shared<bodies::RigidBox>(
      glm::dvec3{500.0, 2.0, 500.0}, 1.0, glm::dvec3{0.0, 50.0, 0.0},
      glm::dquat{{0.0, 0.0, 0.0}},
      std::array{glm::vec3{1.0, 0.0, 0.0}, glm::vec3{1.0, 0.0, 0.0},
                 glm::vec3{0.4, 0.4, 0.4}, glm::vec3{0.4, 0.4, 0.4},
                 glm::vec3{0.0, 1.0, 0.0}, glm::vec3{0.0, 1.0, 0.0}}));
  */

  glutMainLoop();
}

}  // namespace rigid_dynamics