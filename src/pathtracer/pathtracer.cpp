#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out;

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading
  for (int i = 0; i < num_samples; i++) {
      Vector3D wi = hemisphereSampler->get_sample();
      Ray ray = Ray(hit_p, o2w*wi);
      ray.min_t = EPS_F;
      ray.max_t = (r.o - hit_p).norm() - EPS_F;
      Intersection inter;
      if (bvh->intersect(ray, &inter)) {
          double pdf = 1/(2*PI);
          L_out += isect.bsdf->f(w_out, wi) * inter.bsdf->get_emission() * (wi).z/ pdf;
      }
  }
  L_out = L_out / num_samples;

  return L_out;

}

Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  Vector3D L_out;
  int j = 0;
    for (auto light : scene->lights) {
      if (light->is_delta_light()) { //only sample once for point lights
          Vector3D wi;
          double pdf;
          double distToLight;
          Intersection inter;
          Vector3D rad = light->sample_L(hit_p, &wi, &distToLight, &pdf);
          Ray ray = Ray(hit_p, wi);
          ray.min_t = EPS_F;
          ray.max_t = distToLight - EPS_F;
          if (!bvh->intersect(ray, &inter)) {
              L_out += rad * isect.bsdf->f(w_out, w2o*wi) * (w2o * wi).z/pdf;
          }
      } else {
          for (int i = 0; i < ns_area_light; i++) {
              Vector3D wi;
              double distToLight;
              double pdf;
              Intersection inter;
              Vector3D rad = light->sample_L(hit_p, &wi, &distToLight, &pdf);
              Ray ray = Ray(hit_p, wi);
              ray.min_t = EPS_F;
              ray.max_t = distToLight - EPS_F;
              if (!bvh->intersect(ray, &inter)) {
                  L_out += (rad * isect.bsdf->f(w_out, w2o*wi) * (w2o * wi).z/pdf)/ns_area_light;
              }
          }

      }
        j++;
  }

  return L_out/j;

}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light

  return isect.bsdf->get_emission();
}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`

  return direct_hemisphere_sample ? estimate_direct_lighting_hemisphere(r, isect) : estimate_direct_lighting_importance(r, isect);


}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  Vector3D L_out(0, 0, 0);

  // TODO: Part 4, Task 2
  // Returns the one bounce radiance + radiance from extra bounces at this point.
  // Should be called recursively to simulate extra bounces.


  return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.
  //
  // REMOVE THIS LINE when you are ready to begin Part 3.

  if (!bvh->intersect(r, &isect))
    return L_out;


  L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);

  // TODO (Part 3): Return the direct illumination.
  return zero_bounce_radiance(r, isect) + one_bounce_radiance(r, isect);
  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct

  return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  // TODO (Part 1.2):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Vector3D.
  // You should call est_radiance_global_illumination in this function.
  Vector3D average = {0.0,0.0,0.0};
  for (int i = 0; i < ns_aa; i++) {
      Vector2D sample = gridSampler->get_sample();
      Ray ray = camera->generate_ray((x + sample[0])/sampleBuffer.w, (y + sample[1])/sampleBuffer.h);
      average += est_radiance_global_illumination(ray);
  }

  average = average/ns_aa;
  sampleBuffer.update_pixel(average, x, y);

  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"
/*
  int num_samples = ns_aa;          // total samples to evaluate
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel


  sampleBuffer.update_pixel(Vector3D(0.2, 1.0, 0.8), x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = num_samples; */


}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
