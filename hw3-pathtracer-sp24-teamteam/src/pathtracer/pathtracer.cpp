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
    Vector3D wi_world = o2w * wi;
    double pdf = 1.0 / (2 * PI);
    Intersection newIsect;
    Ray sampleRay(hit_p + EPS_F * wi_world, wi_world);

    if (bvh->intersect(sampleRay, &newIsect)) {
      Vector3D reflectedEmission = newIsect.bsdf->get_emission();
      Vector3D light = reflectedEmission * isect.bsdf->f(w_out, wi) * cos_theta(wi) * 2.0 * PI;
      L_out += light;
    }
  }

  return L_out / num_samples;

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

  for (auto light = scene->lights.begin(); light != scene->lights.end(); light++) {
    Vector3D wi_world;
    double distTolight;
    double pdf;
    double cosTheta;
    int numSamples = 0;
    Vector3D sampleLight;
    Vector3D accumLight;

    for (int i = 0; i < ns_area_light; i++) {
      sampleLight = (*light)->sample_L(hit_p, &wi_world, &distTolight, &pdf);
      cosTheta = dot(isect.n, wi_world); // can't just cos_theta(wi_world)

      Ray shadowRay = Ray(hit_p, wi_world, 1);
      shadowRay.min_t = EPS_F; // necessary to set
      shadowRay.max_t = distTolight - EPS_F;
      Intersection newIntersect;
      if (cosTheta > 0 && !(bvh->intersect(shadowRay, &newIntersect))) {
        accumLight += sampleLight * isect.bsdf->f(w_out, wi_world) * cosTheta / pdf;
      }
      numSamples++;

      if ((*light)->is_delta_light()) {
        break;
      }
    }

    L_out += accumLight / numSamples;
  }

  return L_out;

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

  if (direct_hemisphere_sample) {
    return estimate_direct_lighting_hemisphere(r, isect);
  } else {
    return estimate_direct_lighting_importance(r, isect);
  }
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
  

  if (isAccumBounces == 1) {
    L_out += one_bounce_radiance(r, isect);
  } else {
    if (r.depth < max_ray_depth) {
      L_out += one_bounce_radiance(r, isect);
    }
  }
  
  double contProb = 0.6;
  if (coin_flip(contProb)) {
    Vector3D wi;
    double pdf;

    Vector3D bsdf = isect.bsdf->sample_f(w_out, &wi, &pdf);
    Vector3D wi_world = o2w * wi;

    Ray sampleRay(hit_p + EPS_F * wi_world, wi_world);
    sampleRay.depth = r.depth - 1;
    sampleRay.min_t = EPS_F; 

    Intersection newIntersect;
    if (r.depth > 1 && bvh->intersect(sampleRay, &newIntersect)) {
      Vector3D sampleLight = at_least_one_bounce_radiance(sampleRay, newIntersect);
      L_out = sampleLight * bsdf * cos_theta(wi) / pdf / contProb;
    }
  }

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
    return envLight ? envLight->sample_dir(r) : L_out;

  if (isAccumBounces == true) {
    L_out = zero_bounce_radiance(r, isect) + at_least_one_bounce_radiance(r, isect);
  } else {
    L_out = at_least_one_bounce_radiance(r, isect);
  }

  // TODO (Part 3): Return the direct illumination.

  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct

  return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  // TODO (Part 1.2):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Vector3D.
  // You should call est_radiance_global_illumination in this function.
    int num_samples = ns_aa;          // total samples to evaluate
    Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel

    
  
    Vector3D totalRadiance(0, 0, 0);
    double illum;
    double sum_illum = 0;
    double sum_squared_illum = 0;
    double batchSize = 0;
    double mean;
    double stddev;
    double curr_samples = 0;
    double I;

    for (int i = 0; i < num_samples; i++) {
      if (batchSize == samplesPerBatch) {
        batchSize = 0;
        mean = sum_illum / curr_samples;
        stddev = sqrt(1.0 / (curr_samples - 1) * (sum_squared_illum - (sum_illum * sum_illum / curr_samples)));
        I = 1.96 * stddev / sqrt(curr_samples);
        if (I <= maxTolerance * mean) {
          break;
        }
      }

      Vector2D sample = gridSampler->get_sample();
      Ray randomRay = camera->generate_ray((x + sample.x) / sampleBuffer.w, (y + sample.y) / sampleBuffer.h); // normalize
      randomRay.depth = max_ray_depth;

      Vector3D sceneRadiance = est_radiance_global_illumination(randomRay);
      totalRadiance += sceneRadiance;

      illum = sceneRadiance.illum();
      sum_illum += illum;
      sum_squared_illum += illum * illum;

      batchSize++;
      curr_samples++;
    }
    totalRadiance = (1.0 / curr_samples) * totalRadiance;

  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"

  sampleBuffer.update_pixel(totalRadiance, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = curr_samples;
}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
