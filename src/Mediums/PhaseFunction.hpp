#ifndef PHASEFUNCTION_HPP
#define PHASEFUNCTION_HPP

#include "Vector.hpp"
#include "global.hpp"

enum PhaseType {ISOTROPIC, ANISOTROPY};

class PhaseFunction{
public:
    // 在先前采样位置基础上，采样整个球的方向
    // wo出射(朝外)
    inline virtual Vector3f sample(const Vector3f &wo, const Vector3f &position) = 0;

    // 采样方向的pdf
    // wo出射，wi入射(均朝外)
    inline virtual float pdf(const Vector3f &wi, const Vector3f &wo) = 0;

    // 计算phase function
    // wi入射，wo入射，方向均朝外
    inline virtual float eval(const Vector3f &wi, const Vector3f &wo) = 0;
};

class HenyeyGreensteinMedium : public PhaseFunction{
public:
    HenyeyGreensteinMedium(float g) : g(g){}
    inline Vector3f sample(const Vector3f &wo, const Vector3f &position) override;
    inline float pdf(const Vector3f &wi, const Vector3f &wo) override;
    inline float eval(const Vector3f &wi, const Vector3f &wo) override;
private:
    float g;
};

inline Vector3f HenyeyGreensteinMedium::sample(const Vector3f &wo, const Vector3f &position){
    float costheta, phi;
    float costheta_rand = get_random_float(), phi_rand = get_random_float();

    phi = 2 * M_PI * phi_rand;
    if(std::fabs(g) < EPSILON){
        costheta = 1 - 2 * costheta_rand;
    }else{
        float sqrterm = (1 - g * g) / (1 - g + 2 * g * costheta_rand);
        costheta = (1 + g * g - sqrterm * sqrterm) / (2 * g);
    }

    float sintheta = std::sqrt(std::max(0.f, 1 - costheta * costheta));
    Vector3f x, y;
    CoordinateSystem(wo, x, y);
    return sintheta * std::cos(phi) * x + sintheta * std::sin(phi) * y + costheta * (-wo);
}

inline float HenyeyGreensteinMedium::pdf(const Vector3f &wi, const Vector3f &wo){
    return eval(wi, wo);
}

inline float HenyeyGreensteinMedium::eval(const Vector3f &wi, const Vector3f &wo){
    g = clamp(-1.f, 1.f, g);

    float costheta = dotProduct(wi, wo);
    float denom = 1 + g * g + 2 * g * costheta;
    float inv4pi = 1 / (4 * M_PI);
    return inv4pi * (1 - g * g) / (denom * std::sqrt(denom));
}

#endif