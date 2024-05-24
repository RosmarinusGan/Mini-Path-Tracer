#include "Texture.hpp"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

Texture::Texture(const std::string& name) : width(0), height(0), channel(3)
{
    data = stbi_load(name.c_str(), &width, &height, &channel, channel);
    // color_texture = cv::imread(name);
    // cv::cvtColor(color_texture, color_texture, cv::COLOR_BGR2RGB); // ？
    // width = color_texture.cols;
    // height = color_texture.rows;
}

Texture::~Texture()
{
    stbi_image_free(data);
}

// u, v纹理坐标
Vector3f Texture::getColor(float u, float v) const
{
    u = clamp(0.f, 1.f, u);
    v = clamp(0.f, 1.f, v);

    float u_img = u * (width - 1);
    float v_img = (1 - v) * (height - 1);
    // float u_img = u * width >= width ? width - 1 : u * width;
    // float v_img = (1 - v) * height >= height ? height - 1 : (1 - v) * height;
    int u_max = std::ceil(u_img);
    int u_min = std::floor(u_img);
    int v_max = std::ceil(v_img);
    int v_min = std::floor(v_img);

    auto find_color = [&](int u, int v){
        auto color = data + (v * width + u) * channel;
        return Vector3f((float)color[0], (float)color[1], (float)color[2]) / 255.f;
    };

    // bilinear interpolation
    auto color_u = (find_color(u_min, v_min) *(u_max - u_img) + find_color(u_max, v_min) * (u_img - u_min)) / (u_max - u_min);
    auto color_v = (find_color(u_min, v_max) *(u_max - u_img) + find_color(u_max, v_max) * (u_img - u_min)) / (u_max - u_min);
    return (color_u * (v_max - v_img) + color_v * (v_img - v_min)) / (v_max - v_min);
    
    // auto color = color_texture.at<cv::Vec3b>(v_img, u_img);
    // auto color = data + (v_img * width + u_img) * channel;
    // return Vector3f(color[0], color[1], color[2]) / 255.f;
}