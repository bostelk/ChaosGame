#include <Eigen/Core>
#include <cstdio>
#include <vector>

#define _CRT_SECURE_NO_WARNINGS
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

struct Image
{
  // in Bytes.
  Eigen::MatrixXi Pixel;

  // in Pixels.
  int Width;
  int Height;

  Image(int ImageWidth, int ImageHeight)
  {
    Width = ImageWidth;
    Height = ImageHeight;
    Pixel = Eigen::MatrixXi(ImageHeight, ImageWidth);
  }
};

std::vector<std::function<Eigen::Vector2f(Eigen::Vector2f&)>>
Sierpinski()
{
  std::vector<std::function<Eigen::Vector2f(Eigen::Vector2f&)>> functions = {
    [](Eigen::Vector2f& point) {
      Eigen::Vector2f newPoint = point / 2;
      return newPoint;
    },

    [](Eigen::Vector2f& point) {
      Eigen::Vector2f newPoint = (point + Eigen::Vector2f(1, 0)) / 2;
      return newPoint;
    },

    [](Eigen::Vector2f& point) {
      Eigen::Vector2f newPoint = (point + Eigen::Vector2f(0, 1)) / 2;
      return newPoint;
    }
  };
  return functions;
}

Eigen::Vector2f
ChaosGame(
  std::vector<std::function<Eigen::Vector2f(Eigen::Vector2f&)>>& functions,
  int NumIterations = 20)
{
  // A random point in biunit square.
  Eigen::Vector2f point = Eigen::Vector2f::Random();
  for (int i = 0; i < NumIterations; i++) {
    // Pick a random function to iterate.
    int index = rand() % functions.size();
    auto function = functions[index];
    point = function(point);
  }

  return point;
}

int
main(void)
{
  int numPoints = 50000;

  std::vector<Eigen::Vector2f> points;
  points.reserve(numPoints);

  // Iterated functions.
  auto ifs = Sierpinski();

  for (int i = 0; i < numPoints; i++) {
    printf("%i\n", i);
    Eigen::Vector2f point = ChaosGame(ifs);
    points.push_back(point);
  }

  Image image(1024, 1024);

  // Map points to an image.
  for (Eigen::Vector2f& point : points) {
    // To nearest pixel.
    int x = std::lroundf((1.0 + point[0]) / 2.0 * image.Width);
    int y = std::lroundf((1.0 + point[1]) / 2.0 * image.Height);
    image.Pixel(x, y) = 0xff;
  }

  stbi_write_png("fractal.png",
                 image.Width,
                 image.Height,
                 3,
                 image.Pixel.data(),
                 image.Width * 3);

  return 0;
}