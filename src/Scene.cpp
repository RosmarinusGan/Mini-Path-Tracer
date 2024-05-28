#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

// 在所有自发光物体上随机选一个物体，然后在该物体上随机选一个点
void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

// 光线与场景中所有物体求交（被bvh取代）
// bool Scene::trace(
//         const Ray &ray,
//         const std::vector<Object*> &objects,
//         float &tNear, uint32_t &index, Object **hitObject)
// {
//     *hitObject = nullptr;
//     for (uint32_t k = 0; k < objects.size(); ++k) {
//         float tNearK = kInfinity;
//         uint32_t indexK;
//         Vector2f uvK;
//         if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
//             *hitObject = objects[k];
//             tNear = tNearK;
//             index = indexK;
//         }
//     }
//     return (*hitObject != nullptr);
// }

// Implementation of Path Tracing
Vector3f Scene::castRayPT(const Ray &ray) const
{
    //Path Tracing Algorithm
    Intersection inter = Scene::intersect(ray);

    /* volumetric */
    float dis = medium->sample(ray);
    bool hitMedium = dis < inter.distance;
    //inter.distance -= 200.f;
    //std::cout << dis << std::endl;
    //hitMedium = dis < 3.f;
    //hitMedium = false; // 关闭体积光
    /* volumetric */
    
    if(!inter.happened) return this->backgroundColor; // 背景色

    /* volumetric */
    if(!hitMedium){
        // 光线直接打到光源/光线最终到达光源
        if(inter.m->hasEmission()) return inter.m->getEmission();
    }
    //if(inter.m->hasEmission()) return inter.m->getEmission();

    // // 光线直接打到光源/光线最终到达光源
    // if(inter.m->hasEmission()) return inter.m->getEmission();

    auto pos = hitMedium ? ray(dis) : inter.coords; // 散射点 / 着色点位置
    auto n = hitMedium ? -ray.direction : inter.normal.normalized(); // 散射点出射方向 / 着色点法线
    auto wo = -ray.direction; // 出射光方向/视线方向
    Vector3f pos_deviation = (dotProduct(ray.direction, n) < 0) ?
                        pos + n * EPSILON :
                        pos - n * EPSILON ; // 散射点 / 着色点位置偏移
    /* volumetric */                    

    // 直接光照
    Vector3f L_dir = 0.f;

    auto compute_direct = [&]{
        // 对光源均匀采样
        Intersection lightPoint;
        float lightPdf = 0.f;
        sampleLight(lightPoint, lightPdf);

        auto light_pos = lightPoint.coords; // 光源位置
        auto light_n = lightPoint.normal.normalized(); // 光源法线
        auto ws = (light_pos - pos).normalized(); // 指向光源方向

        // 多重重要性采样，对brdf或phase function采样
        Vector3f L_dir_frp = 0.f;
        /* volumetric */
        auto w_mis = hitMedium ? medium->pf->sample(wo, pos).normalized() : inter.m->sample(wo, n).normalized();  // 散射光方向 / 入射光方向
        /* volumetric */
        // 方向是否朝向光源
        Intersection shadow_mis = Scene::intersect(Ray(pos_deviation, w_mis));
        bool mis_IsHitLight = shadow_mis.happened && shadow_mis.m->hasEmission();
        float frpPdf = hitMedium ? medium->pf->pdf(w_mis, wo) : inter.m->pdf(w_mis, wo, n);
        if(mis_IsHitLight){
            if(!hitMedium){
                auto fr = inter.m->eval(w_mis, wo, n, inter.tcoords);
                auto costheta = dotProduct(w_mis, n);
                L_dir_frp = shadow_mis.emit * fr * costheta / frpPdf;
            }else{
                auto fp = medium->pf->eval(w_mis, wo);
                L_dir_frp = shadow_mis.emit * fp / frpPdf;
            }
        }
        
        Vector3f L_dir_light = 0.f;
        // 判断是否遮挡
        Ray shade_to_light(pos_deviation, ws);
        //Ray shade_to_light(pos, ws);
        Intersection shadowInter = Scene::intersect(shade_to_light);
        auto dis_shadeToLight = (light_pos - pos).norm();
        auto dis_shadeToLight2 = dotProduct((light_pos - pos), (light_pos - pos));
        auto costheta_prime = dotProduct(-ws, light_n);

        // 光线一定会打到光源点上，除非被遮挡
        // 这里的判断精度不能太高，否则会出现奇怪的阴影
        if(shadowInter.happened && fabs(shadowInter.distance - dis_shadeToLight) < 0.01){
            // 计算直接光照
            auto Li = lightPoint.emit;
            /* volumetric */
            if(!hitMedium){
                auto fr = inter.m->eval(ws, wo, n, inter.tcoords);
                auto costheta = dotProduct(ws, n);
                L_dir_light = Li * fr * costheta * costheta_prime / (dis_shadeToLight2 * lightPdf);
            }else{
                auto fp = medium->pf->eval(ws, wo);
                L_dir_light = Li * fp * costheta_prime / (dis_shadeToLight2 * lightPdf);
            }
            /* volumetric */
        }

        float lightPdf_mis = dis_shadeToLight2 * lightPdf / costheta_prime;
        // beta = 2
        float omega_frp = frpPdf * frpPdf / (frpPdf * frpPdf + lightPdf_mis * lightPdf_mis);
        float omega_light = lightPdf_mis * lightPdf_mis / (frpPdf * frpPdf + lightPdf_mis * lightPdf_mis);
        L_dir = L_dir_frp * omega_frp + L_dir_light * omega_light;
        //L_dir = L_dir_light;

        /* volumetric */
        //L_dir = medium->Tr(dis_shadeToLight) * L_dir; // 这段要不要乘上Tr？似乎不用，因为最后结果乘了coeff
        /* volumetric */
    };

    if(!hitMedium){
        switch(inter.m->getType()){
            case MIRROR:
            {
                L_dir = 0.f; // 因为完全镜面反射只对一个方向采样，就不单独考虑直接光照，而是一并在间接光照中计算
                break; 
            }

            default:
            {
                compute_direct();
                break;
            }
        }
    }else{
        compute_direct();
    }
    
    // 间接光照
    Vector3f L_indir = 0.f;
    float ksi = get_random_float();
    //ksi = 1.f; //只算直接光照
    if(ksi < RussianRoulette){
        /* volumetric */
        auto wi = hitMedium ? medium->pf->sample(wo, pos).normalized() : inter.m->sample(wo, n).normalized();  // 散射光方向 / 入射光方向
        /* volumetric */
        // auto wi = normalize(input_pos - pos); // 这样计算是错误的，原因:sample得到的就是方向（从着色点出发），不是位置（不是从原点出发）

        Ray traceRay(pos_deviation, wi);
        //Ray traceRay(pos, wi);
        Intersection traceInter = Scene::intersect(traceRay);

        auto compute_indirect = [&]{
            /* volumetric */
            if(!hitMedium){
                auto fr = inter.m->eval(wi, wo, n, inter.tcoords);
                auto costheta = dotProduct(wi, n);
                auto inputPdf = inter.m->pdf(wi, wo, n);
                // 入射光在半球内,否则当pdf接近0时，会出现白色噪点
                // 这是合理的，因为像素收敛是正确的，但采样数不够，根据能量守恒，为了弥补未采样到的点，会出现高亮白色噪点（firefly），本质是采样数不够
                if(inputPdf > EPSILON)
                    L_indir = castRayPT(traceRay) * fr * costheta / (inputPdf * RussianRoulette);
            }else{
                auto fp = medium->pf->eval(wi, wo);
                auto inputPdf = medium->pf->pdf(wi, wo);
                //if(inputPdf > EPSILON)
                L_indir = castRayPT(traceRay) * fp / (inputPdf * RussianRoulette);
            }
            /* volumetric */
        };

        //if(!traceInter.happened) L_indir = backgroundColor; // 错误
        if(!traceInter.happened) 
            L_indir = backgroundColor; // 如果没有交点，意味着色点没有收到间接光照(是0还是背景色？)
            // L_indir = 0.f;
        else{
            /* volumetric */
            if(!hitMedium){
                switch (inter.m->getType()){
                    case MIRROR:
                    {
                        compute_indirect();
                        break;
                    }

                    default:
                    {
                        if(!traceInter.m->hasEmission()){
                            compute_indirect();
                        }
                        break;
                    }
                }
            }else{
                if(!traceInter.m->hasEmission()){
                    compute_indirect();
                }
                // L_dir = 0.f;
                // compute_indirect();
            }
            /* volumetric */
        }
    }

    //return L_dir + L_indir;

    /* volumetric */
    // 乘上对距离采样的系数
    //if(hitMedium){L_indir += Vector3f(1.0f, 0.78f, 0.78f);}
    float coeff = hitMedium ? medium->coefficient(dis, inter.distance) : 1.f;
    return coeff * (L_dir + L_indir);
    /* volumetric */
}

 Vector3f Scene::castRayBasic(const Ray &ray) const
 {
    Intersection inter = Scene::intersect(ray);
    if(!inter.happened) return this->backgroundColor; // 背景色

    auto pos = inter.coords; // 着色点位置
    auto n = inter.normal.normalized(); // 着色点法线
    auto wo = -ray.direction; // 视线方向
    Vector3f pos_deviation = (dotProduct(ray.direction, n) < 0) ?
                        pos + n * EPSILON :
                        pos - n * EPSILON ; // 着色点位置偏移

    Vector3f hitColor = 0.f;

    // 光线打到光源直接返回
    if(inter.m->hasEmission()) return inter.m->getEmission();
    // 光线打到diffuse物体返回blinn-phong着色
    if(inter.m->getType() == DIFFUSE || inter.m->getType() == MICROFACET){
        // 对每个光源都计算贡献
        for (uint32_t k = 0; k < objects.size(); ++k){
            if(objects[k]->hasEmit())
            {
                Intersection light_point;
                float pdf;
                objects[k]->Sample(light_point, pdf);
                auto light_pos = light_point.coords; // 光源位置
                auto light_dir = (light_pos - pos).normalized(); // 指向光源方向
                auto light_I = light_point.emit;
                auto dis_shadeToLight = (light_pos - pos).norm();
                auto dis_shadeToLight2 = dotProduct(dis_shadeToLight, dis_shadeToLight);
                int amplitude = 10000;
                Intersection shadowInter = Scene::intersect(Ray(pos_deviation, light_dir));
                if(shadowInter.happened && fabs(shadowInter.distance - dis_shadeToLight) < 0.01){
                    Vector3f Ld = inter.m->getColorAt(inter.tcoords.x, inter.tcoords.y) * light_I * std::max(0.f, dotProduct(n, light_dir)) / dis_shadeToLight2;
                    Vector3f half = (light_dir + wo).normalized();
                    Vector3f Ls = inter.m->Ks * light_I * std::pow(std::max(0.f, dotProduct(half, n)), inter.m->specularExponent) / dis_shadeToLight2;
                    //hitColor += Ld + Ls + La;
                    hitColor += (Ld + Ls) * amplitude;
                }
            }
        }
        return hitColor;
    }

    float ksi = get_random_float();
    if(ksi < RussianRoulette){
        Vector3f reflectDir, refractDir;
        inter.m->getReflectRefract(wo, n, reflectDir, refractDir);   

        if(inter.m->getType() == MIRROR){
            hitColor = castRayBasic(Ray(pos_deviation, reflectDir));
            return hitColor * inter.m->pdf(reflectDir, wo, n);
        }

        if(inter.m->getType() == TRANSPARENT){
            Vector3f reflectColor = 0.f, refractColor = 0.f;
            // 计算反射
            reflectColor = castRayBasic(Ray(pos_deviation, reflectDir));
            // 计算折射
            refractColor = castRayBasic(Ray(pos_deviation, refractDir));
            // Fresnel
            float kr = inter.m->pdf(reflectDir, wo, n);
            hitColor = reflectColor * kr + refractColor * (1 - kr);
            return hitColor;
        }
    }

    return hitColor;
 }

// // 处理区域光源
// std::tuple<Vector3f, Vector3f> HandleAreaLight(const AreaLight &light, const Vector3f &hitPoint, const Vector3f &N,
//                                                    const Vector3f &shadowPointOrig,
//                                                    const std::vector<Object *> &objects, uint32_t &index,
//                                                    const Vector3f &dir, float specularExponent)
//                                                    {
//     // TO DO Implement Area Light Here

//                                                    }