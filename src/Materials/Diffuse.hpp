#pragma once
#include "Material.hpp"

class Diffuse : public Material
{
public:
    Diffuse(Vector3f kd = Vector3f(1.0f), Vector3f e = Vector3f(0.f));
    Vector3f sample(const Vector3f &wo, const Vector3f &N) override;
    float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N) override;
    Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N, const Vector2f &tcoord) override;
    ~Diffuse() = default;
};

Diffuse::Diffuse(Vector3f kd, Vector3f e) : Material(DIFFUSE, e){
    this->Kd = kd;
}

// refer: pbrt
Vector3f Diffuse::sample(const Vector3f &wo, const Vector3f &N){
    // uniform sample on the hemisphere
    // float x_1 = get_random_float(), x_2 = get_random_float();
    // float z = std::fabs(1.0f - 2.0f * x_1); // 随机半球方向的z轴坐标，z在[0,1]之间
    // float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2; // r是随机半球方向投影到xy平面的长度，phi是随机半球方向绕法线旋转的角度
    // Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
    // return toWorld(localRay, N);
    
    // 针对disk的concentric采样
    float random_x = get_random_float(), random_y = get_random_float();
    Vector2f point = Vector2f(random_x, random_y) * 2.f + Vector2f(-1.f);
    if(point.x == 0 && point.y == 0) point = Vector2f(0.f);
    float theta, r;
    if(std::fabs(point.x) > std::fabs(point.y)){
        r = point.x;
        theta = (M_PI / 4) * (point.y / point.x);
    }
    else{
        r = point.y;
        theta = (M_PI / 2) - (M_PI / 4) * (point.x / point.y);
    }
    point = Vector2f(r * std::cos(theta), r * std::sin(theta));

    // 投影至半球上
    float z = std::sqrt(std::max(0.f, 1 - point.x * point.x - point.y * point.y));
    return toWorld(Vector3f(point.x, point.y, z), N);
}

float Diffuse::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    // // uniform sample probability 1 / (2 * PI)
    // if (dotProduct(wi, N) > 0.0f)
    //     return 0.5f / M_PI;
    // else
    //     return 0.0f;
    // break;
    if(dotProduct(wi, N) > 0.0f)
        return dotProduct(wi, N) / M_PI;
    else 
        return 0.0f;
}

Vector3f Diffuse::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N, const Vector2f &tcoord){
    float cosalpha = dotProduct(N, wi);
    if (cosalpha > 0.0f) {
        // Vector3f diffuse = Kd / M_PI;
        Vector3f diffuse = getColorAt(tcoord.x, tcoord.y) / M_PI;
        return diffuse;
    }
    else
        return Vector3f(0.0f);
}