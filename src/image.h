#pragma once

#include <Eigen/Core>
#include <vector>

// A 24-bit color.
struct Color24
{
    union
    {
        uint8_t Data[3];
        uint8_t R;
        uint8_t G;
        uint8_t B;
    };

    Color24(uint8_t R, uint8_t G, uint8_t B)
    {
        Data[0] = R;
        Data[1] = G;
        Data[2] = B;
    }

    // Normalized.
    static Color24 Color24f(float R, float G, float B) { return Color24(std::lroundf(R * 255), std::lroundf(G * 255), std::lroundf(B * 255)); }
};

extern const Color24 White;
extern const Color24 Red;
extern const Color24 Green;
extern const Color24 Blue;
extern const Color24 Black;

struct Pixel
{
    Eigen::Vector2i Coord;
    int Density;
    Color24 Color;
    Pixel()
        : Coord(Eigen::Vector2i::Zero())
        , Density(0)
        , Color(White)
    {}
    Pixel(Eigen::Vector2i coord)
        : Coord(coord)
        , Density(0)
        , Color(White)
    {}
};

struct Image
{
    // in Bytes.
    uint8_t* Data;

    // Red, Blue, Green.
    int Channels = 3;

    size_t SizeBytes;
    size_t ColStrideBytes;
    size_t RowStrideBytes;

    // in Pixels.
    int Width;
    int Height;

    Image(int ImageWidth, int ImageHeight)
        : Width(ImageWidth)
        , Height(ImageHeight)
        , SizeBytes(ImageHeight* ImageWidth* Channels)
        , ColStrideBytes(Channels)
        , RowStrideBytes(ImageWidth* Channels)
        , Data(nullptr)
    {}
    Image() : Image(0, 0) {}

    // Allocate data buffer.
    void Allocate()
    {
        Data = (uint8_t*)malloc(SizeBytes);
        memset(Data, 0, SizeBytes);
    }

    // Free data buffer.
    void Release() { free(Data); }

    size_t GetOffsetBytes(int x, int y)
    {
        size_t OffsetBytes = x * ColStrideBytes + y * RowStrideBytes;
        assert(OffsetBytes < SizeBytes);
        return OffsetBytes;
    }

    // Write a color to all pixels.
    void Clear(Color24 color)
    {
        for (int y = 0; y < Height; y++) {
            for (int x = 0; x < Width; x++) {
                size_t OffsetBytes = GetOffsetBytes(x, y);
                memcpy(Data + OffsetBytes, &color.Data, sizeof(Color24));
            }
        }
    }

    // Write pixels to an image.
    void WritePixels(const std::vector<Pixel>& pixels)
    {
        for (const Pixel& pixel : pixels) {
            size_t OffsetBytes = GetOffsetBytes(pixel.Coord[0], pixel.Coord[1]);
            memcpy(Data + OffsetBytes, &pixel.Color.Data, sizeof(Color24));
        }
    }
    
    // Write image to PNG file.
    int WriteFile(std::string filename);
};

