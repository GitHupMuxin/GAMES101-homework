//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

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

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection shadeInter = intersect(ray);
    if(!shadeInter.happened)
        return {};
    if (shadeInter.m->hasEmission())
    {
        if (!depth)
            return shadeInter.m->getEmission();
        return Vector3f(0);
    }
    Vector3f x = shadeInter.coords;
    Vector3f N = shadeInter.normal.normalized();
    Vector3f wo = ray.direction;
    Intersection lightInter;
    Vector3f L_dir;
    float pdf_light = 0;
    sampleLight(lightInter, pdf_light);
    Vector3f pToX = lightInter.coords - shadeInter.coords;
    Vector3f NN = lightInter.normal.normalized();
    Vector3f ws = pToX.normalized();
    Ray targetToLight(x, ws);
    Intersection blockPoint = intersect(targetToLight);
    if (shadeInter.m->m_type != GLASS)
        if(blockPoint.happened && pToX.norm() - blockPoint.distance  < EPSILON)
            L_dir = lightInter.emit * shadeInter.m->eval(-wo, ws, N) * dotProduct(ws, N) * dotProduct(-ws, NN) / dotProduct(pToX, pToX) / pdf_light;
    if (get_random_float() > this->RussianRoulette)
        return L_dir;
    Vector3f L_indir = 0;
    Vector3f wi = shadeInter.m->sample(ray.direction, N).normalized();
    L_indir = castRay(Ray(shadeInter.coords, wi), depth + 1) * shadeInter.m->eval(-wo, wi, N) * dotProduct(wi, N) / shadeInter.m->pdf(-wo, wi, N) / this->RussianRoulette;
    return L_dir + L_indir;
}