#ifndef HOMOMEDIUM_HPP
#define HOMOMEDIUM_HPP

#include "Medium.hpp"
#include "PhaseFunction.hpp"

class HomoMedium : public Medium{
public:
    HomoMedium(const float &sigma_a, const float &sigma_s, PhaseFunction *pf) 
    : Medium(HOMOMEDIUM, pf), sigma_a(sigma_a), sigma_s(sigma_s), sigma_t(sigma_a + sigma_s){}
    ~HomoMedium() = default;
    inline float Tr(const float &distance) override;
    inline float sample(const Ray &ray) override;
    inline float coefficient(const float &distance, const float &max_distance) override;

private:
    float sigma_a, sigma_s, sigma_t;
};

inline float HomoMedium::Tr(const float &distance){
    return std::exp(-sigma_t * distance);
}

inline float HomoMedium::sample(const Ray &ray){
    float t = -std::log(1 - get_random_float()) / sigma_t;
    return t;
}

inline float HomoMedium::coefficient(const float &distance, const float &max_distance){
    float pdf_medium = sigma_t * Tr(distance);
    float pdf_surface = std::exp(-sigma_t * max_distance);
    float coeff_medium = sigma_s * Tr(distance) / pdf_medium;
    float coeff_surf = Tr(max_distance) / pdf_surface;
    return distance < max_distance ? coeff_medium : coeff_surf;
}

#endif