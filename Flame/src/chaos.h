#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include "image.h"

Color24
IdentityColorMap(const Pixel& p, const Image& image, int numPoints);

Color24
PositionColorMap(const Pixel& p, const Image& image, int numPoints);

Color24
DensityColorMap(const Pixel& p, const Image& image, int numPoints);

std::vector<std::function<Eigen::Vector2f(Eigen::Vector2f&)>> Sierpinski();

std::vector<std::function<Eigen::Vector2f(Eigen::Vector2f&)>>
AffineTransformations(std::vector<Eigen::Affine2f> transforms);

std::vector<std::function<Eigen::Vector2f(Eigen::Vector2f&)>>
CurryAll(std::vector<std::function<Eigen::Vector2f(Eigen::Vector2f&)>> functions, std::function<Eigen::Vector2f(Eigen::Vector2f&)> func);

Eigen::Vector2f
IdentityFunc(Eigen::Vector2f& point);

Eigen::Vector2f
SinusoidalFunc(Eigen::Vector2f& point);

Eigen::Vector2f
SphericalFunc(Eigen::Vector2f& point);

Eigen::Vector2f
SwirlFunc(Eigen::Vector2f& point);

Eigen::Vector2f
HorseshoeFunc(Eigen::Vector2f& point);

Eigen::Vector2f
PolarFunc(Eigen::Vector2f& point);

Eigen::Vector2f
HandkerchiefFunc(Eigen::Vector2f& point);

Eigen::Vector2f
HeartFunc(Eigen::Vector2f& point);

Eigen::Vector2f
DiscFunc(Eigen::Vector2f& point);

Eigen::Vector2f
SpiralFunc(Eigen::Vector2f& point);

Eigen::Vector2f
HyperbolicFunc(Eigen::Vector2f& point);

Eigen::Vector2f
DiamondFunc(Eigen::Vector2f& point);

Eigen::Vector2f
exFunc(Eigen::Vector2f& point);

Eigen::Vector2f
JuliaFunc(Eigen::Vector2f& point);

void SetTRS(float* translation, float rotation, float* scale);

void
RenderImage(Image image,
    std::vector<std::function<Eigen::Vector2f(Eigen::Vector2f&)>> ifs,
    std::function<Color24(const Pixel& p, const Image& image, int numPoints)> colorMap,
    int numPoints,
    int numIterations);

void
RenderAnimation(std::string filename, std::function<Color24(const Pixel& p, const Image& image, int numPoints)> colorMap, int numPoints, int imageWidth, int imageHeight, int numFrames);