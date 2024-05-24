#pragma once

#include "Material.hpp"

class Transparent : public Material
{
public:
    Transparent(float ior, Vector3f ks, Vector3f e = Vector3f(0.f)) : Material(TRANSPARENT, e) {
        this->ior = ior;
        this->Ks = ks;
    };
    ~Transparent() = default;
    inline Vector3f sample(const Vector3f &wo, const Vector3f &N) override{};
    // 返回fresnel系数
    inline float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N) override
    {
        float kr;
        fresnel(-wo, N, ior, kr);
        return kr;
    }
    inline Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N, const Vector2f &tcoord) override{};
};
