#pragma once
#include "Material.hpp"
#include "Vector.hpp"
#include "global.hpp"

class Microfacet : public Material{
public:
    Microfacet(Vector3f kd = Vector3f(1.0f), float roughness = 1.0f, Vector3f e = Vector3f(0.f));
    inline Vector3f sample(const Vector3f &wo, const Vector3f &N) override;
    inline float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N) override;
    inline Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N, const Vector2f &tcoord) override;
    virtual ~Microfacet() = default;
};

Microfacet::Microfacet(Vector3f kd, float r, Vector3f e) : Material(MICROFACET, e){
    this->Kd = kd;
    this->roughness = r;
}

inline Vector3f Microfacet::sample(const Vector3f &wo, const Vector3f &N){
    // 重要性采样
    float random_phi = get_random_float(), random_theta = get_random_float();
    float phi = 2 * M_PI * random_phi, theta = std::atan(roughness * std::sqrt(random_theta / (1 - random_theta)));
    Vector3f local_wh = Vector3f(std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta));
    Vector3f sample_wh = toWorld(local_wh, N).normalized();
    return -reflect(wo, sample_wh);
}

inline float Microfacet::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    // 这里用的是立体角的pdf，然后转换为入射角的pdf
    // refer: https://agraphicsguynotes.com/posts/sample_microfacet_brdf/
    if(dotProduct(wi, N) > 0.0f)
    {
        
        Vector3f wh = normalize(wi + wo);
        float costheta_h = dotProduct(wh, N);
        float transformer = 4.0f * dotProduct(wo, wh);
        //if(roughness <= EPSILON) return 1.0f; // 完美镜面
        float pdf_wh = roughness * roughness * costheta_h / (M_PI * std::pow((roughness * roughness - 1) * costheta_h * costheta_h + 1, 2));
        return pdf_wh / transformer;
    }
    else 
        return 0.0f;
}

//refer: pbrt
inline Vector3f Microfacet::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N, const Vector2f &tcoord){
    float cosalpha = dotProduct(N, wi);
    if(cosalpha > 0.0f){
        Vector3f F;
        float G, D;
        // 计算Fresnel项
        F = schlick(-wo, N, tcoord);
        //fresnel(-wo, N, ior, F);

        // 计算几何项
        auto Geometry_ggx = [&](const float &roughness, const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
            float tantheta_wi = std::tan(acos(dotProduct(wi, N)));
            float tantheta_wo = std::tan(acos(dotProduct(wo, N)));
            if(std::isinf(tantheta_wi) || std::isinf(tantheta_wo)) return 0.0f;

            float G_wi = (-1 + std::sqrt(1 + roughness * roughness * tantheta_wi * tantheta_wi)) / 2;
            float G_wo = (-1 + std::sqrt(1 + roughness * roughness * tantheta_wo * tantheta_wo)) / 2;

            float result = 1 / (1 + G_wi + G_wo);
            //result = clamp(0.0f, 1.0f, result); //clamp后接近镜面情况会全黑？
            return result;
        };
        G = Geometry_ggx(roughness, wi, wo, N);

        // 计算法线分布项
        auto Distribution_ggx = [&](const float &roughness, const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
            Vector3f wh = normalize(wi + wo); // half vector
            float costheta_h = dotProduct(N, wh);
            float sintheta_h = std::sin(acos(costheta_h));
            float tantheta_h = sintheta_h / costheta_h;
            if(std::isinf(tantheta_h)) return 0.0f;

            //if(roughness <= EPSILON) return 1.0f; // 完美镜面

            float result = (roughness * roughness) / (M_PI * std::pow(roughness * roughness * costheta_h * costheta_h + sintheta_h * sintheta_h, 2));
            //result = clamp(0.0f, 1.0f, result);  //clamp后接近镜面情况会全黑？
            return result;
        };
        D = Distribution_ggx(roughness, wi, wo, N);

        // 归一化常数
        float norm = 4 * std::abs(dotProduct(N, wi)) * std::abs(dotProduct(N, wo));

        
        // Vector3f fm = Vector3f(F * G * D / norm);
        Vector3f fm = F * G * D / norm;

        // 漫反射项（工业做法不够学术，TODO：kulla-conty）
        //Vector3f fd = (Vector3f(1.f) - F) * Kd / M_PI;
        Vector3f fd = (Vector3f(1.f) - F) * getColorAt(tcoord.x, tcoord.y) / M_PI;

        return fd + fm;
    }
    else return Vector3f(0.0f);
}