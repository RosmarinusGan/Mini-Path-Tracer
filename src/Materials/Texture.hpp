#pragma once

#include "Vector.hpp"
#include "global.hpp"

class Texture{
public:
    Texture(const std::string& name);
    virtual ~Texture();
    Vector3f getColor(float u, float v) const;
private:
    // cv::Mat color_texture;
    unsigned char* data;
    int width, height, channel;
};