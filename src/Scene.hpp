#pragma once

#include "BVH.hpp"
#include "Vector.hpp"
#include "Object.hpp"
#include "Light.hpp"
#include "AreaLight.hpp"
#include "Ray.hpp"
#include "Medium.hpp"
#include "HomoMedium.hpp"
#include "PhaseFunction.hpp"

class Scene
{
public:
    // setting up options
    int width = 1024;
    int height = 1024;
    double fov = 40;
    //Vector3f backgroundColor = Vector3f(0.235294, 0.67451, 0.843137);
    Vector3f backgroundColor = 0.f;
    Vector3f La = Vector3f(0.1f, 0.1f, 0.1f);
    float RussianRoulette = 0.8; // RR概率
    
    std::unique_ptr<PhaseFunction> phase = std::make_unique<HenyeyGreensteinMedium>(0.7f);
    std::unique_ptr<Medium> medium = std::make_unique<HomoMedium>(0.00025f, 0.0003f, phase.get());

    Scene(int w, int h) : width(w), height(h) {}

    void Add(Object *object) { objects.push_back(object); }
    void Add(std::unique_ptr<Light> light) { lights.push_back(std::move(light)); }

    const std::vector<Object*>& get_objects() const { return objects; }
    const std::vector<std::unique_ptr<Light> >&  get_lights() const { return lights; }
    Intersection intersect(const Ray& ray) const;

    BVHAccel *bvh;
    void buildBVH();

    Vector3f castRayPT(const Ray &ray) const; //path tracing
    Vector3f castRayBasic(const Ray &ray) const; // whitted-style
    void sampleLight(Intersection &pos, float &pdf) const;
    // bool trace(const Ray &ray, const std::vector<Object*> &objects, float &tNear, uint32_t &index, Object **hitObject);
    
    std::tuple<Vector3f, Vector3f> HandleAreaLight(const AreaLight &light, const Vector3f &hitPoint, const Vector3f &N,
                                                   const Vector3f &shadowPointOrig,
                                                   const std::vector<Object *> &objects, uint32_t &index,
                                                   const Vector3f &dir, float specularExponent);

    // creating the scene (adding objects and lights)
    std::vector<Object* > objects;
    std::vector<std::unique_ptr<Light> > lights;

};