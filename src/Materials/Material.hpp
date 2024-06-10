#pragma once

#include "Vector.hpp"
#include "global.hpp"
#include "Texture.hpp"

enum MaterialType {DIFFUSE, MIRROR, MICROFACET, TRANSPARENT}; // 待扩展

class Material{
protected:
    // Compute reflection direction
    // I是入射方向（朝外），返回出射光方向（朝内）
    Vector3f reflect(const Vector3f &I, const Vector3f &N) const{
        return I - 2 * dotProduct(I, N) * N;
    }

    // Compute refraction direction using Snell's law
    // I是入射方向（朝内）,返回折射光方向（朝外）
    Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) const{
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }

    // Compute Fresnel equation
    // I是入射方向（朝内），kr是返回的Fresnel系数
    // TODO: (天花板有高光)是否要用shclick方法？返回是一维的，是否要改进，返回Vector3f类型
    void fresnel(const Vector3f &I, const Vector3f &N, const float &ior, float &kr) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi > 0) {  std::swap(etai, etat); }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1) {
            kr = 1;
        }
        else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is given by:
        // kt = 1 - kr;
    }

    Vector3f schlick(const Vector3f &I, const Vector3f &N, const Vector2f &tcoord) const{
        // float cosi = fabs(dotProduct(I, N));
        // float R0 = std::pow((1 - ior) / (1 + ior), 2);
        // return R0 + (1 - R0) * std::pow(1 - cosi, 5);
        float cosi = fabs(dotProduct(I, N));
        return getColorAt(tcoord.x, tcoord.y) + (Vector3f(1.0f) - getColorAt(tcoord.x, tcoord.y)) * std::pow(1 - cosi, 5);
    }

    // a是法线表示为（0，0，1）的局部坐标系下的坐标，给定法线在世界坐标系下的坐标N，返回a在世界坐标系下的坐标
    Vector3f toWorld(const Vector3f &a, const Vector3f &N){
        Vector3f B, C;
        // // 条件判断防止除0，也即叉积出来不是0向量
        // if (std::fabs(N.x) > std::fabs(N.y)){
        //     float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
        //     C = Vector3f(N.z * invLen, 0.0f, -N.x *invLen); // C是N与向量(0,1,0)的叉积，并且归一化的结果
        // }
        // else {
        //     float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
        //     C = Vector3f(0.0f, N.z * invLen, -N.y *invLen); // C是N与向量(1,0,0)的叉积，并且归一化的结果
        // }
        // B = crossProduct(C, N);
        CoordinateSystem(N, B, C);
        return a.x * B + a.y * C + a.z * N;
    }

public:
    MaterialType m_type;
    //Vector3f m_color;
    Vector3f m_emission;
    float ior;
    Vector3f Kd, Ks;
    float specularExponent;
    float roughness;
    std::unique_ptr<Texture> texture_color;

    //inline Material(MaterialType t = DIFFUSE, Vector3f e = Vector3f(0.f), float r = 1.0f);
    inline Material(MaterialType t = DIFFUSE, Vector3f e = Vector3f(0.f));
    inline MaterialType getType(){return m_type;}
    //inline Vector3f getColor(){return m_color;}
    inline Vector3f getColorAt(float u, float v) const;
    inline Vector3f getEmission(){return m_emission;}
    inline bool hasEmission();

    // 输入wo出射方向（朝外），获得反射方向reflect和折射方向refract(朝外)
    inline void getReflectRefract(const Vector3f &wo,const Vector3f &N, Vector3f &reflect, Vector3f &refract)
    {
        reflect = -this->reflect(wo, N).normalized();
        refract = this->refract(-wo, N, ior).normalized();
    }

    // sample a ray by Material properties
    // 在半球上随机采样一个方向，就是局部坐标就是以着色点为原点，返回世界坐标系下的坐标（也即还没有归一化的方向）
    // wo出射方向
    inline virtual Vector3f sample(const Vector3f &wo, const Vector3f &N) = 0;

    // given a ray, calculate the PdF of this ray
    // 采样的pdf
    // 半球方向采样的pdf(1/2pi),如果采样点在半球下方，返回0
    // wo出射，wi入射(均朝外)
    inline virtual float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N) = 0;

    // given a ray, calculate the contribution of this ray
    // 计算BRDF
    // wo出射，wi入射（均朝外）
    inline virtual Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N, const Vector2f &tcoord) = 0;

    virtual ~Material() = default;

    // 设置纹理
    void setTexture(const std::string& name){
        texture_color = std::make_unique<Texture>(name);
    }
};

Material::Material(MaterialType t, Vector3f e){
    m_type = t;
    //m_color = c;
    m_emission = e;
    //roughness = r;
}

// 如果m_emission的模大于EPSILON，就认为这个材质有发光
inline bool Material::hasEmission() {
    if (m_emission.norm() > EPSILON) return true;
    else return false;
}

inline Vector3f Material::getColorAt(float u, float v) const{
    if(texture_color == nullptr || u < 0 || v < 0) return Kd;
    return texture_color->getColor(u, v);
}

// // 在半球上随机采样一个方向，就是局部坐标就是以着色点为原点，返回世界坐标系下的坐标（也即还没有归一化的方向）
// // wo出射方向
// Vector3f Material::sample(const Vector3f &wo, const Vector3f &N){
//     switch(m_type){
//         // refer: pbrt
//         case DIFFUSE:
//         {
//             // uniform sample on the hemisphere
//             // float x_1 = get_random_float(), x_2 = get_random_float();
//             // float z = std::fabs(1.0f - 2.0f * x_1); // 随机半球方向的z轴坐标，z在[0,1]之间
//             // float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2; // r是随机半球方向投影到xy平面的长度，phi是随机半球方向绕法线旋转的角度
//             // Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
//             // return toWorld(localRay, N);
            
//             // 针对disk的concentric采样
//             float random_x = get_random_float(), random_y = get_random_float();
//             Vector2f point = Vector2f(random_x, random_y) * 2.f + Vector2f(-1.f);
//             if(point.x == 0 && point.y == 0) point = Vector2f(0.f);
//             float theta, r;
//             if(std::fabs(point.x) > std::fabs(point.y)){
//                 r = point.x;
//                 theta = (M_PI / 4) * (point.y / point.x);
//             }
//             else{
//                 r = point.y;
//                 theta = (M_PI / 2) - (M_PI / 4) * (point.x / point.y);
//             }
//             point = Vector2f(r * std::cos(theta), r * std::sin(theta));

//             // 投影至半球上
//             float z = std::sqrt(std::max(0.f, 1 - point.x * point.x - point.y * point.y));
//             return toWorld(Vector3f(point.x, point.y, z), N);

//             break;
//         }
        
//         // 只采集反射方向
//         case MIRROR:
//         {
//             return -reflect(wo, N);
//             break;
//         }

//         case MICROFACET:
//         {
//             // // uniform sample on the hemisphere
//             // float x_1 = get_random_float(), x_2 = get_random_float();
//             // float z = std::fabs(1.0f - 2.0f * x_1); // 随机半球方向的z轴坐标，z在[0,1]之间
//             // float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2; // r是随机半球方向投影到xy平面的长度，phi是随机半球方向绕法线旋转的角度
//             // Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
//             // return toWorld(localRay, N);

//             // 重要性采样
//             float random_phi = get_random_float(), random_theta = get_random_float();
//             float phi = 2 * M_PI * random_phi, theta = std::atan(roughness * std::sqrt(random_theta / (1 - random_theta)));
//             Vector3f local_wh = Vector3f(std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta));
//             Vector3f sample_wh = toWorld(local_wh, N).normalized();
//             return -reflect(wo, sample_wh);
//             break;
//         }
//     }
// }

// // 采样的pdf
// // 半球方向采样的pdf(1/2pi),如果采样点在半球下方，返回0
// // wo出射，wi入射(均朝外)
// float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
//     switch(m_type){
//         case DIFFUSE:
//         {
//             // // uniform sample probability 1 / (2 * PI)
//             // if (dotProduct(wi, N) > 0.0f)
//             //     return 0.5f / M_PI;
//             // else
//             //     return 0.0f;
//             // break;

            
//             if(dotProduct(wi, N) > 0.0f)
//                 return dotProduct(wi, N) / M_PI;
//             else 
//                 return 0.0f;
//         }

//         case MIRROR:
//         {
//             if(dotProduct(wi, N) > 0.0f)
//                 return 1.0f;
//             else 
//                 return 0.0f;
//             break;
//         }

//         case MICROFACET:
//         {
//             // // uniform sample probability 1 / (2 * PI)
//             // if (dotProduct(wi, N) > 0.0f)
//             //     return 0.5f / M_PI;
//             // else
//             //     return 0.0f;
//             // break;
            
//             // 这里用的是立体角的pdf，然后转换为入射角的pdf
//             // refer: https://agraphicsguynotes.com/posts/sample_microfacet_brdf/
//             if(dotProduct(wi, N) > 0.0f)
//             {
                
//                 Vector3f wh = normalize(wi + wo);
//                 float costheta_h = dotProduct(wh, N);
//                 float transformer = 4.0f * dotProduct(wo, wh);
//                 if(roughness == 0.0f) return 1.0f / transformer; // 完美镜面
//                 float pdf_wh = roughness * roughness * costheta_h / (M_PI * std::pow((roughness * roughness - 1) * costheta_h * costheta_h + 1, 2));
//                 return pdf_wh / transformer;
//             }
//             else 
//                 return 0.0f;
//             break;
//         }
//     }
// }

// // 计算BRDF
// // wo出射，wi入射（均朝外）
// Vector3f Material::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N, const Vector2f &tcoord){
//     switch(m_type){
//         case DIFFUSE:
//         {
//             // calculate the contribution of diffuse model
//             float cosalpha = dotProduct(N, wi);
//             if (cosalpha > 0.0f) {
//                 // Vector3f diffuse = Kd / M_PI;
//                 Vector3f diffuse = getColorAt(tcoord.x, tcoord.y) / M_PI;
//                 return diffuse;
//             }
//             else
//                 return Vector3f(0.0f);
//             break;
//         }

//         case MIRROR:
//         {
//             float cosalpha = dotProduct(N, wi);
//             // float kr; // 返回的Fresnel系数
//             if (cosalpha > 0.0f){
//                 //fresnel(-wo, N, ior, kr);
//                 auto kr = schlick(-wo, N, tcoord);
//                 auto mirror = 1 / cosalpha;
//                 return kr * mirror;
//             }
//             else
//                 return Vector3f(0.0f);
//             break;
//         }

//         //refer: pbrt
//         case MICROFACET:
//         {
//             float cosalpha = dotProduct(N, wi);
//             if(cosalpha > 0.0f){
//                 float F, G, D;
//                 // 计算Fresnel项
//                 fresnel(-wo, N, ior, F);

//                 // 计算几何项
//                 auto Geometry_ggx = [&](const float &roughness, const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
//                     float tantheta_wi = std::tan(acos(dotProduct(wi, N)));
//                     float tantheta_wo = std::tan(acos(dotProduct(wo, N)));
//                     if(std::isinf(tantheta_wi) || std::isinf(tantheta_wo)) return 0.0f;

//                     float G_wi = (-1 + std::sqrt(1 + roughness * roughness * tantheta_wi * tantheta_wi)) / 2;
//                     float G_wo = (-1 + std::sqrt(1 + roughness * roughness * tantheta_wo * tantheta_wo)) / 2;

//                     float result = 1 / (1 + G_wi + G_wo);
//                     clamp(0.0f, 1.0f, result);
//                     return result;
//                 };
//                 G = Geometry_ggx(roughness, wi, wo, N);

//                 // 计算法线分布项
//                 auto Distribution_ggx = [&](const float &roughness, const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
//                     Vector3f wh = normalize(wi + wo); // half vector
//                     float costheta_h = dotProduct(N, wh);
//                     float sintheta_h = std::sin(acos(costheta_h));
//                     float tantheta_h = sintheta_h / costheta_h;
//                     if(std::isinf(tantheta_h)) return 0.0f;

//                     float result = (roughness * roughness) / (M_PI * std::pow(roughness * roughness * costheta_h * costheta_h + sintheta_h * sintheta_h, 2));
//                     clamp(0.0f, 1.0f, result);
//                     return result;
//                 };
//                 D = Distribution_ggx(roughness, wi, wo, N);

//                 // 归一化常数
//                 float norm = 4 * std::abs(dotProduct(N, wi)) * std::abs(dotProduct(N, wo));

//                 if(roughness == 0.0f) D = 1.0f; // 完美镜面
//                 Vector3f fm = Vector3f(F * G * D / norm);

//                 // 漫反射项（工业做法不够学术，TODO：kulla-conty）
//                 Vector3f fd = (1 - F) * Kd / M_PI;

//                 return fd + fm;
//             }
//             else return Vector3f(0.0f);
//             break;
//         }
//     }
// }
