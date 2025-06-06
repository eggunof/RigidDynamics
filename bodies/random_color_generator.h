
#ifndef RIGIDDYNAMICS_BODIES_RANDOM_COLOR_GENERATOR_H_
#define RIGIDDYNAMICS_BODIES_RANDOM_COLOR_GENERATOR_H_

#include <array>
#include <glm/glm.hpp>
#include <random>

class RandomColorGenerator {
  std::mt19937 rng_;

 public:
  explicit RandomColorGenerator(unsigned long seed = std::random_device{}());

  glm::vec3 GenerateRandomColor(float min = 0.0f, float max = 1.0f) {
    std::uniform_real_distribution<float> dist(min, max);
    return {dist(rng_), dist(rng_), dist(rng_)};
  }

  template <size_t N>
  std::array<glm::vec3, N> GenerateRandomColors(float min = 0.0f,
                                                float max = 1.0f) {
    std::array<glm::vec3, N> colors;
    std::uniform_real_distribution<float> dist(min, max);
    for (auto& color : colors) {
      color = glm::vec3(dist(rng_), dist(rng_), dist(rng_));
    }
    return colors;
  }
};

#endif  // RIGIDDYNAMICS_BODIES_RANDOM_COLOR_GENERATOR_H_
