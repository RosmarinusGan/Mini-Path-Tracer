#pragma once
#include <iostream>
#include <cmath>
#include <random>

#undef M_PI
#define M_PI 3.141592653589793f
#define GAMMA_C 2.2f

extern const float EPSILON;
const float kInfinity = std::numeric_limits<float>::max();

inline float clamp(const float &lo, const float &hi, const float &v)
{ return std::max(lo, std::min(hi, v)); }

inline bool solveQuadratic(const float &a, const float &b, const float &c, float &x0, float &x1)
{
    float discr = b * b - 4 * a * c;
    if (discr < 0) return false;
    else if (discr == 0) x0 = x1 = - 0.5 * b / a;
    else {
        float q = (b > 0) ?
                  -0.5 * (b + sqrt(discr)) :
                  -0.5 * (b - sqrt(discr));
        x0 = q / a;
        x1 = c / q;
    }
    if (x0 > x1) std::swap(x0, x1);
    return true;
}

// windows下需要加static，修改后能显著提高效率
// 0-1
inline float get_random_float()
{
    static std::random_device dev; // 随机设备dev，以dev生成的随机数为随机种子
    static std::mt19937 rng(dev()); // 使用梅森旋转算法生成伪随机数，以dev为种子
    static std::uniform_real_distribution<float> dist(0.f, 1.f); // 均匀实数分布，范围[0,1]的浮点数

    return dist(rng); // 使用rng作为随机数源，从dist分布中抽取一个随机数
}

inline void UpdateProgress(float progress)
{
    int barWidth = 70;

    std::cout << "[";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << " %\r";
    std::cout.flush();
}

// 给定一个单位向量v1，构建一个坐标系
inline void CoordinateSystem(const Vector3f &v1, Vector3f &v2, Vector3f &v3)
{
    if(std::fabs(v1.x) > std::fabs(v1.y)){
        v2 = Vector3f(-v1.z, 0, v1.x) / std::sqrt(v1.x * v1.x + v1.z * v1.z);
    }else{
        v2 = Vector3f(0, v1.z, -v1.y) / std::sqrt(v1.y * v1.y + v1.z * v1.z);
    }
    v3 = crossProduct(v1, v2);
}


