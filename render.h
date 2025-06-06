
#ifndef RIGIDDYNAMICS_RENDER_H_
#define RIGIDDYNAMICS_RENDER_H_

#include <GL/freeglut_std.h>

#include <glm/glm.hpp>
#include <string>

namespace rigid_dynamics {
;
static int WIDTH = 800;
static int HEIGHT = 600;

void Run(int argc, char* argv[]);

void DrawText(const glm::dvec2& position, const std::string& text,
              const glm::dvec3& color = {1.0, 1.0, 1.0},
              void* font = GLUT_BITMAP_HELVETICA_18);
void Display();
void Reshape(int width, int height);
void Idle();
void Keyboard(unsigned char key, int x, int y);
void KeyboardUp(unsigned char key, int x, int y);
void PassiveMotion(int x, int y);
void Mouse(int button, int state, int x, int y);

}  // namespace rigid_dynamics

#endif  // RIGIDDYNAMICS_RENDER_H_
