#include "image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

int Image::WriteFile(std::string filename)
{
    return stbi_write_png(filename.c_str(), Width, Height, Channels, Data, RowStrideBytes);
}