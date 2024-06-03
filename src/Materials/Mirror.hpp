#pragma once

#include "Material.hpp"

class Mirror : public Material{
public:
    Mirror(const float ior = 10.f, Vector3f e = Vector3f(0.f));
    Vector3f sample(const Vector3f &wo, const Vector3f &N) override;
    float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N) override;
    Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N, const Vector2f &tcoord) override;
    ~Mirror() = default;
};

Mirror::Mirror(const float ior, Vector3f e) : Material(MIRROR, e){
    this->ior = ior;
}

Vector3f Mirror::sample(const Vector3f &wo, const Vector3f &N){
    return -reflect(wo, N);
}

float Mirror::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    if(dotProduct(wi, N) > 0.0f && dotProduct(wo, N) > 0.0f)
        return 1.0f;
    else 
        return 0.0f;
}

Vector3f Mirror::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N, const Vector2f &tcoord){
    float cosalpha_i = dotProduct(N, wi);
    float cosalpha_o = dotProduct(N, wo);
    Vector3f kr; // 返回的Fresnel系数
    if (cosalpha_i > 0.0f && cosalpha_o > 0.0f){
        // 如果不是镜面反射直接返回0
        if(crossProduct(wi + wo, N).norm() < EPSILON){
            //fresnel(-wo, N, ior, kr);
            kr = schlick(-wo, N, tcoord);
            auto mirror = 1 / cosalpha_i;
            return kr * mirror;
        }
    }
    return Vector3f(0.f);
}