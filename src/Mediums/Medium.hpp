#ifndef MEDIUM_HPP
#define MEDIUM_HPP

#include "Vector.hpp"
#include "PhaseFunction.hpp"
#include "Ray.hpp"

enum MediumType{HOMOMEDIUM, INHOMOMEDIUM};

class Medium{
protected:
    MediumType type;
public:
    Medium(MediumType type, PhaseFunction *pf) : type(type), pf(pf) {}
    ~Medium() = default;

    PhaseFunction* pf;

    inline MediumType getType() {return type;}
    // ray should be unoccluded and fully contained in the medium
    //virtual inline Vector3f Tr(const Ray &ray) = 0;
    // 返回透射率
    inline virtual float Tr(const float &distance) = 0;

    //沿ray采样距离
    inline virtual float sample(const Ray &ray) = 0;

    //返回radiance的系数（包括要除的距离采样的pdf）
    //virtual inline float pdf(const Ray &ray) = 0;
    inline virtual float coefficient(const float &distance, const float &max_distance) = 0;
};

#endif