#include "chaos.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <unordered_map>
#include <vector>

// Microsoft Concurrency
#include <concurrent_unordered_map.h>
#include <concurrent_vector.h>
#include <ppl.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// clang-format off

// Copyright 2019 Google LLC.
// SPDX-License-Identifier: Apache-2.0

// Author: Anton Mikhailov

// The look-up tables contains 256 entries. Each entry is a an sRGB triplet.

unsigned char turbo_srgb_bytes[256][3] = { {48,18,59},{50,21,67},{51,24,74},{52,27,81},{53,30,88},{54,33,95},{55,36,102},{56,39,109},{57,42,115},{58,45,121},{59,47,128},{60,50,134},{61,53,139},{62,56,145},{63,59,151},{63,62,156},{64,64,162},{65,67,167},{65,70,172},{66,73,177},{66,75,181},{67,78,186},{68,81,191},{68,84,195},{68,86,199},{69,89,203},{69,92,207},{69,94,211},{70,97,214},{70,100,218},{70,102,221},{70,105,224},{70,107,227},{71,110,230},{71,113,233},{71,115,235},{71,118,238},{71,120,240},{71,123,242},{70,125,244},{70,128,246},{70,130,248},{70,133,250},{70,135,251},{69,138,252},{69,140,253},{68,143,254},{67,145,254},{66,148,255},{65,150,255},{64,153,255},{62,155,254},{61,158,254},{59,160,253},{58,163,252},{56,165,251},{55,168,250},{53,171,248},{51,173,247},{49,175,245},{47,178,244},{46,180,242},{44,183,240},{42,185,238},{40,188,235},{39,190,233},{37,192,231},{35,195,228},{34,197,226},{32,199,223},{31,201,221},{30,203,218},{28,205,216},{27,208,213},{26,210,210},{26,212,208},{25,213,205},{24,215,202},{24,217,200},{24,219,197},{24,221,194},{24,222,192},{24,224,189},{25,226,187},{25,227,185},{26,228,182},{28,230,180},{29,231,178},{31,233,175},{32,234,172},{34,235,170},{37,236,167},{39,238,164},{42,239,161},{44,240,158},{47,241,155},{50,242,152},{53,243,148},{56,244,145},{60,245,142},{63,246,138},{67,247,135},{70,248,132},{74,248,128},{78,249,125},{82,250,122},{85,250,118},{89,251,115},{93,252,111},{97,252,108},{101,253,105},{105,253,102},{109,254,98},{113,254,95},{117,254,92},{121,254,89},{125,255,86},{128,255,83},{132,255,81},{136,255,78},{139,255,75},{143,255,73},{146,255,71},{150,254,68},{153,254,66},{156,254,64},{159,253,63},{161,253,61},{164,252,60},{167,252,58},{169,251,57},{172,251,56},{175,250,55},{177,249,54},{180,248,54},{183,247,53},{185,246,53},{188,245,52},{190,244,52},{193,243,52},{195,241,52},{198,240,52},{200,239,52},{203,237,52},{205,236,52},{208,234,52},{210,233,53},{212,231,53},{215,229,53},{217,228,54},{219,226,54},{221,224,55},{223,223,55},{225,221,55},{227,219,56},{229,217,56},{231,215,57},{233,213,57},{235,211,57},{236,209,58},{238,207,58},{239,205,58},{241,203,58},{242,201,58},{244,199,58},{245,197,58},{246,195,58},{247,193,58},{248,190,57},{249,188,57},{250,186,57},{251,184,56},{251,182,55},{252,179,54},{252,177,54},{253,174,53},{253,172,52},{254,169,51},{254,167,50},{254,164,49},{254,161,48},{254,158,47},{254,155,45},{254,153,44},{254,150,43},{254,147,42},{254,144,41},{253,141,39},{253,138,38},{252,135,37},{252,132,35},{251,129,34},{251,126,33},{250,123,31},{249,120,30},{249,117,29},{248,114,28},{247,111,26},{246,108,25},{245,105,24},{244,102,23},{243,99,21},{242,96,20},{241,93,19},{240,91,18},{239,88,17},{237,85,16},{236,83,15},{235,80,14},{234,78,13},{232,75,12},{231,73,12},{229,71,11},{228,69,10},{226,67,10},{225,65,9},{223,63,8},{221,61,8},{220,59,7},{218,57,7},{216,55,6},{214,53,6},{212,51,5},{210,49,5},{208,47,5},{206,45,4},{204,43,4},{202,42,4},{200,40,3},{197,38,3},{195,37,3},{193,35,2},{190,33,2},{188,32,2},{185,30,2},{183,29,2},{180,27,1},{178,26,1},{175,24,1},{172,23,1},{169,22,1},{167,20,1},{164,19,1},{161,18,1},{158,16,1},{155,15,1},{152,14,1},{149,13,1},{146,11,1},{142,10,1},{139,9,2},{136,8,2},{133,7,2},{129,6,2},{126,5,2},{122,4,3} };

// clang-format on

const float PI = 3.14159f;

float
Saturate01(float v)
{
    // Min.
    if (v < 0) {
        return 0;
    }
    // Max.
    if (v > 1) {
        return 1;
    }
    return v;
}

struct CoordKey
{
    int x;
    int y;
    CoordKey(int x, int y)
        : x(x)
        , y(y)
    {}
};

bool
operator==(const CoordKey& lhs, const CoordKey& rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

template<>
struct std::hash<CoordKey>
{
    std::size_t operator()(CoordKey const& p) const noexcept
    {
        std::size_t h1 = std::hash<int>{}(p.x);
        std::size_t h2 = std::hash<int>{}(p.y);
        return h1 ^ (h2 << 1); // or use boost::hash_combine
    }
};

const Color24 White = Color24(255, 255, 255);
const Color24 Red = Color24(255, 0, 0);
const Color24 Green = Color24(0, 255, 0);
const Color24 Blue = Color24(0, 0, 255);
const Color24 Black = Color24(0, 0, 0);

// Try hilbert curve.
Color24
IdentityColorMap(const Pixel& p, const Image& image, int numPoints)
{
    return p.Color;
};

// Try hilbert curve.
Color24
PositionColorMap(const Pixel& p, const Image& image, int numPoints)
{
    float xNormalized = Saturate01(p.Coord[0] / (float)image.Width);
    float yNormalized = Saturate01(p.Coord[1] / (float)image.Height);
    return Color24::Color24f(xNormalized, yNormalized, 0);
};

Color24
DensityColorMap(const Pixel& p, const Image& image, int numPoints)
{
#if _DEBUG
    printf("Density: %i\n", p.Density);
#endif
    float densityNormalized = Saturate01(p.Density / (float)20); // numPoints;
    int indexLUT = std::lroundf(densityNormalized * 255);
    return Color24(turbo_srgb_bytes[indexLUT][0], turbo_srgb_bytes[indexLUT][1], turbo_srgb_bytes[indexLUT][2]);
};

std::vector<std::function<Eigen::Vector2f(Eigen::Vector2f&)>>
Sierpinski()
{
    std::vector<std::function<Eigen::Vector2f(Eigen::Vector2f&)>> functions = { [](Eigen::Vector2f& point) {
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
                                                                                } };
    return functions;
}

std::vector<std::function<Eigen::Vector2f(Eigen::Vector2f&)>>
AffineTransformations(std::vector<Eigen::Affine2f> transforms)
{
    std::vector<std::function<Eigen::Vector2f(Eigen::Vector2f&)>> functions;

    for (const auto& transform : transforms) {
        auto function = [transform](Eigen::Vector2f& point) {
            Eigen::Vector2f newPoint = transform * point;
            return newPoint;
        };
        functions.push_back(function);
    }
    return functions;
}

std::vector<std::function<Eigen::Vector2f(Eigen::Vector2f&)>>
CurryAll(std::vector<std::function<Eigen::Vector2f(Eigen::Vector2f&)>> functions, std::function<Eigen::Vector2f(Eigen::Vector2f&)> func)
{
    std::vector<std::function<Eigen::Vector2f(Eigen::Vector2f&)>> newFunctions;
    newFunctions.reserve(functions.size());

    for (std::function<Eigen::Vector2f(Eigen::Vector2f&)> function : functions) {
        std::function<Eigen::Vector2f(Eigen::Vector2f&)> newFunction = [function, func](Eigen::Vector2f& point) {
            Eigen::Vector2f newPoint = function(point);
            return func(newPoint);
        };
        newFunctions.push_back(newFunction);
    }
    return newFunctions;
}

Eigen::Vector2f
IdentityFunc(Eigen::Vector2f& point)
{
    return point;
}

Eigen::Vector2f
SinusoidalFunc(Eigen::Vector2f& point)
{
    Eigen::Vector2f newPoint(sin(point[0]), sin(point[1]));
    return newPoint;
}

Eigen::Vector2f
SphericalFunc(Eigen::Vector2f& point)
{
    float r = point.stableNorm();
    float r2 = r * r;
    Eigen::Vector2f newPoint(point[0] / r2, point[1] / r2);
    return newPoint;
}

Eigen::Vector2f
SwirlFunc(Eigen::Vector2f& point)
{
    float r = point.stableNorm();
    float theta = atan(point[1] / point[0]);
    Eigen::Vector2f newPoint(r * cos(theta + r), r * sin(theta + r));
    return newPoint;
}

Eigen::Vector2f
HorseshoeFunc(Eigen::Vector2f& point)
{
    float r = point.stableNorm();
    float theta = atan(point[1] / point[0]);
    Eigen::Vector2f newPoint(r * cos(2 * theta), r * sin(2 * theta));
    return newPoint;
}

Eigen::Vector2f
PolarFunc(Eigen::Vector2f& point)
{
    float r = point.stableNorm();
    float theta = atan(point[1] / point[0]);
    Eigen::Vector2f newPoint(theta / PI, r - 1);
    return newPoint;
}

Eigen::Vector2f
HandkerchiefFunc(Eigen::Vector2f& point)
{
    float r = point.stableNorm();
    float theta = atan(point[1] / point[0]);
    Eigen::Vector2f newPoint(r * sin(theta + r), r * cos(theta - r));
    return newPoint;
}

Eigen::Vector2f
HeartFunc(Eigen::Vector2f& point)
{
    float r = point.stableNorm();
    float theta = atan(point[1] / point[0]);
    Eigen::Vector2f newPoint(r * sin(theta * r), -r * cos(theta * r));
    return newPoint;
}

Eigen::Vector2f
DiscFunc(Eigen::Vector2f& point)
{
    float r = point.stableNorm();
    float theta = atan(point[1] / point[0]);
    Eigen::Vector2f newPoint(theta * sin(r * PI) / PI, theta * cos(r * PI) / PI);
    return newPoint;
};

Eigen::Vector2f
SpiralFunc(Eigen::Vector2f& point)
{
    float r = point.stableNorm();
    float theta = atan(point[1] / point[0]);
    Eigen::Vector2f newPoint(r * cos(theta + r), r * sin(theta + r));
    return newPoint;
}

Eigen::Vector2f
HyperbolicFunc(Eigen::Vector2f& point)
{
    float r = point.stableNorm();
    float theta = atan(point[1] / point[0]);
    Eigen::Vector2f newPoint((cos(theta) + sin(r)) / r, (cos(theta) - sin(r)) / r);
    return newPoint;
}

Eigen::Vector2f
DiamondFunc(Eigen::Vector2f& point)
{
    float r = point.stableNorm();
    float theta = atan(point[1] / point[0]);
    Eigen::Vector2f newPoint(sin(theta) * cos(r), cos(theta) * sin(r));
    return newPoint;
}

Eigen::Vector2f
exFunc(Eigen::Vector2f& point)
{
    float r = point.stableNorm();
    float theta = atan(point[1] / point[0]);
    Eigen::Vector2f newPoint(r * cos(theta + r), r * sin(theta + r));
    return newPoint;
}

Eigen::Vector2f
JuliaFunc(Eigen::Vector2f& point)
{
    float r = point.stableNorm();
    float theta = atan(point[1] / point[0]);
    float omega = (rand() % 1) ? 0 : PI; // random either 0 or pi (complex square root branch).
    Eigen::Vector2f newPoint(sqrt(r) * cos(theta / 2 + omega), sqrt(r) * sin(theta / 2 + omega));
    return newPoint;
}

Eigen::Vector2f
ChaosGame(std::vector<std::function<Eigen::Vector2f(Eigen::Vector2f&)>>& functions, Eigen::Vector2f point, int* random, int numIterations = 20)
{
    for (int i = 0; i < numIterations; i++) {
        // Pick a random function to iterate.
        int index = random[i] % functions.size();
        auto function = functions[index];
        point = function(point);
    }

    return point;
}

void
RenderImage(std::string filename,
    std::vector<std::function<Eigen::Vector2f(Eigen::Vector2f&)>> ifs,
    std::function<Color24(const Pixel& p, const Image& image, int numPoints)> colorMap,
    int numPoints,
    int imageWidth,
    int imageHeight)
{
    auto start_image = std::chrono::high_resolution_clock::now();

    printf("Render image: %s\n", filename.c_str());

    concurrency::concurrent_vector<Eigen::Vector2f> points;
    points.reserve(numPoints);

    int numIterations = 20;

    std::vector<int> random;
    random.resize(numIterations * numPoints);

    auto start_t = std::chrono::high_resolution_clock::now();

    // Generate entropy in a single thread.
    for (int i = 0; i < numPoints; i++) {
        // A random point in biunit square [-1,1].
        Eigen::Vector2f point = Eigen::Vector2f::Random();
        points.push_back(point);

        for (int j = 0; j < numIterations; j++) {
            random[i * numIterations + j] = rand();
        }
    }

    auto end_t = std::chrono::high_resolution_clock::now();
    auto int_s = std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t);
    std::cout << "  Generated entropy in " << int_s.count() << " milliseconds." << std::endl;

    start_t = std::chrono::high_resolution_clock::now();

    concurrency::parallel_for(size_t(0), size_t(numPoints), [&ifs, &points, &random, &numIterations](size_t i) {
#if _DEBUG
        printf("Sample point: %ld\n", i);
#endif
        points[i] = ChaosGame(ifs, points[i], random.data() + i * numIterations, numIterations);
    });

    end_t = std::chrono::high_resolution_clock::now();
    int_s = std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t);
    std::cout << "  Sampled points in " << int_s.count() << " milliseconds." << std::endl;

    Image image(imageWidth, imageHeight);
    image.Allocate();

    concurrency::concurrent_unordered_map<CoordKey, Pixel> pixelMap;

    start_t = std::chrono::high_resolution_clock::now();

    // Map points to pixels.
    concurrency::parallel_for_each(begin(points), end(points), [&pixelMap, &image](const Eigen::Vector2f point) {
        // Map from [-1,1] to [0,1].
        Eigen::Vector2f newPoint = ((Eigen::Vector2f(1.0, 1.0) + point) / 2);

        // Clamp point to bounds.
        for (int i = 0; i < newPoint.size(); i++) {
            newPoint(i) = Saturate01(newPoint[i]);
        }

        // Round to nearest pixel.
        int x = std::lroundf(newPoint[0] * (image.Width - 1));
        int y = std::lroundf(newPoint[1] * (image.Height - 1));

        CoordKey key(x, y);

        if (pixelMap.count(key) > 0) {
            pixelMap[key].Density += 1;
        }
        else {
            pixelMap[key] = Pixel(Eigen::Vector2i(x, y));
        }
    });

    end_t = std::chrono::high_resolution_clock::now();
    int_s = std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t);
    std::cout << "  Mapped points to pixels in " << int_s.count() << " milliseconds." << std::endl;

    // Populate pixels.
    std::vector<Pixel> pixels;
    pixels.reserve(pixelMap.size());

    for (auto& kv : pixelMap) {
        pixels.push_back(kv.second);
    }

    start_t = std::chrono::high_resolution_clock::now();

    // Map pixels to a color.
    for (Pixel& pixel : pixels) {
        pixel.Color = colorMap(pixel, image, numPoints);
    }

    end_t = std::chrono::high_resolution_clock::now();
    int_s = std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t);
    std::cout << "  Colored pixels in " << int_s.count() << " milliseconds." << std::endl;

    start_t = std::chrono::high_resolution_clock::now();

    image.WritePixels(pixels);

    stbi_write_png(filename.c_str(), image.Width, image.Height, image.Channels, image.Data, image.RowStrideBytes);

    end_t = std::chrono::high_resolution_clock::now();
    int_s = std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t);
    std::cout << "  Write image to file in " << int_s.count() << " milliseconds." << std::endl;

    image.Release();

    end_t = std::chrono::high_resolution_clock::now();
    auto int_ms = std::chrono::duration_cast<std::chrono::seconds>(end_t - start_image);
    std::cout << "Rendered in " << int_ms.count() << " seconds." << std::endl;
}

void
RenderAnimation(std::string filename, std::function<Color24(const Pixel& p, const Image& image, int numPoints)> colorMap, int numPoints, int imageWidth, int imageHeight, int numFrames)
{
    float theta = 0.0;

    printf("Render animation: %i\n", numFrames);

    for (int frameIndex = 0; frameIndex < numFrames; frameIndex++) {
        Eigen::Affine2f t0;
        t0.matrix() << 0.562482f, 0.397861f, -0.539599f, 0.501088, -.42992, -.112404, 0, 0, 1;
        Eigen::Affine2f t1;
        t1.matrix() << 0.830039, -0.496174, 0.16248, 0.750468, 0.91022, 0.288389, 0, 0, 1;

        Eigen::Rotation2D<float> rot2(theta * PI);
        t0.rotate(rot2);
        t1.rotate(rot2);

        char buffer[2048];
        memset(buffer, 0, 2048);

        snprintf(buffer, 2048, "output/frame%04d.png", frameIndex);

        RenderImage(std::string(buffer, 2048), CurryAll(AffineTransformations({ t0, t1 }), SphericalFunc), colorMap, numPoints, imageWidth, imageHeight);

        std::cout << "Rendered animate frame " << "(" << frameIndex << "/" << numFrames << ").";

        theta += 0.0001;
    }
}
