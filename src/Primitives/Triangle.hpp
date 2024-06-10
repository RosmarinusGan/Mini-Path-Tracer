#pragma once

#include "Intersection.hpp"
#include "Material.hpp"
#include "Object.hpp"

class Triangle : public Object
{
public:
    Vector3f v0, v1, v2; // vertices A, B ,C , counter-clockwise order
    Vector3f n0, n1, n2;
    Vector3f e1, e2;     // 2 edges v1-v0, v2-v0;
    Vector2f t0, t1, t2; // texture coords
    Vector3f normal;
    float area;
    Material* m;

    Triangle(Vector3f _v0, Vector3f _v1, Vector3f _v2, Material* _m = nullptr)
        : v0(_v0), v1(_v1), v2(_v2), m(_m)
    {
        e1 = v1 - v0;
        e2 = v2 - v0;
        normal = normalize(crossProduct(e1, e2));
        // 这样决定的法线认为顺时针方向的是正面，用于进行背面剔除

        area = crossProduct(e1, e2).norm()*0.5f;
    }

    // bool intersect(const Ray& ray) override;
    // bool intersect(const Ray& ray, float& tnear,
    //                uint32_t& index) const override;
    Intersection getIntersection(Ray ray) override;
    // void getSurfaceProperties(const Vector3f& P, const Vector3f& I,
    //                           const uint32_t& index, const Vector2f& uv,
    //                           Vector3f& N, Vector2f& st) const override
    // {
    //     N = normal;
    //     //        throw std::runtime_error("triangle::getSurfaceProperties not
    //     //        implemented.");
    // }
    // Vector3f evalDiffuseColor(const Vector2f&) const override;
    Bounds3 getBounds() override;

    // 对三角形采样，返回采样点和pdf
    void Sample(Intersection &pos, float &pdf){
        // 在三角形面积中随机采样一点，以其重心坐标表示
        float x = std::sqrt(get_random_float()), y = get_random_float();
        pos.coords = v0 * (1.0f - x) + v1 * (x * (1.0f - y)) + v2 * (x * y);
        pos.normal = this->normal;
        pdf = 1.0f / area;
    }
    float getArea(){
        return area;
    }
    bool hasEmit(){
        return m->hasEmission();
    }
};


// inline bool Triangle::intersect(const Ray& ray) { return true; }
// inline bool Triangle::intersect(const Ray& ray, float& tnear,
//                                 uint32_t& index) const
// {
//     return false;
// }

// 三角形的包围盒
inline Bounds3 Triangle::getBounds() { return Union(Bounds3(v0, v1), v2); }

inline Intersection Triangle::getIntersection(Ray ray)
{
    Intersection inter;

    // 背向光线的交点
    // 但是对于basic ray tracing，因为折射我们需要计算背向光线的交点
    // if (dotProduct(ray.direction, normal) > 0)
    //     return inter;
    float u, v, t_tmp = 0;
    Vector3f pvec = crossProduct(ray.direction, e2);
    float det = dotProduct(e1, pvec);
    if (fabs(det) < EPSILON)
        return inter;

    float det_inv = 1. / det;
    Vector3f tvec = ray.origin - v0;
    u = dotProduct(tvec, pvec) * det_inv;
    if (u < 0 || u > 1)
        return inter;
    Vector3f qvec = crossProduct(tvec, e1);
    v = dotProduct(ray.direction, qvec) * det_inv;
    if (v < 0 || u + v > 1)
        return inter;
    t_tmp = dotProduct(e2, qvec) * det_inv;

    if(t_tmp < 0) return inter; // 如果不进行方向与法线的判断，即考虑背向光线的交点，但是一定要去除t小于0的情况

    // TODO find ray triangle intersection
    // u, v重心坐标
    inter.happened = true;
    inter.coords = ray(t_tmp);
    inter.normal = (n0 * (1 - u - v) + n1 * u + n2 * v).normalized();
    //inter.normal = normal;
    inter.obj = this;
    inter.distance = t_tmp;
    inter.emit = m->getEmission();
    inter.m = m;
    inter.tcoords = t0 * (1 - u - v) + t1 * u + t2 * v;

    return inter;
}

// inline Vector3f Triangle::evalDiffuseColor(const Vector2f&) const
// {
//     return Vector3f(0.5, 0.5, 0.5);
// }
