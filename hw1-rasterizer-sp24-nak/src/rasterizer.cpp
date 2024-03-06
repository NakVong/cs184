#include "rasterizer.h"

using namespace std;

namespace CGL {

  RasterizerImp::RasterizerImp(PixelSampleMethod psm, LevelSampleMethod lsm,
    size_t width, size_t height,
    unsigned int sample_rate) {
    this->psm = psm;
    this->lsm = lsm;
    this->width = width;
    this->height = height;
    this->sample_rate = sample_rate;

    sample_buffer.resize(width * height * sample_rate, Color::White);
  }

  // Used by rasterize_point and rasterize_line
  void RasterizerImp::fill_pixel(size_t x, size_t y, Color c) {
    // TODO: Task 2: You might need to this function to fix points and lines (such as the black rectangle border in test4.svg)
    // NOTE: You are not required to implement proper supersampling for points and lines
    // It is sufficient to use the same color for all supersamples of a pixel for points and lines (not triangles)
    for (int i = 0; i < sample_rate; i++) {
      sample_buffer[sample_rate * (y * width + x) + i] = c; // Modified for supersampling
    }
  }

  // Used by rasterize_triangle and rasterize_interpolated_color_triangle
  // 
    void RasterizerImp::fill_supersampled(size_t x, size_t y, size_t index, Color c) {
    int sx = (int)floor(x);
    int sy = (int)floor(y);

    // check bounds
    if (sx < 0 || sx >= width) return;
    if (sy < 0 || sy >= height) return;

    sample_buffer[sample_rate * (y * width + x) + index] = c;
  }



  // Rasterize a point: simple example to help you start familiarizing
  // yourself with the starter code.
  //
  void RasterizerImp::rasterize_point(float x, float y, Color color) {
    // fill in the nearest pixel
    int sx = (int)floor(x);
    int sy = (int)floor(y);

    // check bounds
    if (sx < 0 || sx >= width) return;
    if (sy < 0 || sy >= height) return;

    fill_pixel(sx, sy, color);
    return;
  }

  // Rasterize a line.
  void RasterizerImp::rasterize_line(float x0, float y0,
    float x1, float y1,
    Color color) {
    if (x0 > x1) {
      swap(x0, x1); swap(y0, y1);
    }

    float pt[] = { x0,y0 };
    float m = (y1 - y0) / (x1 - x0);
    float dpt[] = { 1,m };
    int steep = abs(m) > 1;
    if (steep) {
      dpt[0] = x1 == x0 ? 0 : 1 / abs(m);
      dpt[1] = x1 == x0 ? (y1 - y0) / abs(y1 - y0) : m / abs(m);
    }

    while (floor(pt[0]) <= floor(x1) && abs(pt[1] - y0) <= abs(y1 - y0)) {
      rasterize_point(pt[0], pt[1], color);
      pt[0] += dpt[0]; pt[1] += dpt[1];
    }
  }

  bool inside(float Ax, float Ay,
            float Bx, float By,
            float Cx, float Cy,
            float Px, float Py);

  // Rasterize a triangle.
  void RasterizerImp::rasterize_triangle(float x0, float y0,
    float x1, float y1,
    float x2, float y2,
    Color color) {
    // TODO: Task 1: Implement basic triangle rasterization here, no supersampling
    // Vector points
    Vector3D p0(x0, y0, 0);
    Vector3D p1(x1, y1, 0);
    Vector3D p2(x2, y2, 0);

    // Right-hand rule for checking whether winding direction is not ccw
    if (cross(p1 - p0, p2 - p0).z < 0) {
      p1.x = x2;
      p1.y = y2;
      p2.x = x1;
      p2.y = y1;
    }

    // Borders
    vector<float> xList = { x0, x1, x2 };
    vector<float> yList = { y0, y1, y2 };
    int maxX = (ceil(*max_element(xList.begin(), xList.end())));
    int minX = (floor(*min_element(xList.begin(), xList.end())));
    int maxY = (ceil(*max_element(yList.begin(), yList.end())));
    int minY = (floor(*min_element(yList.begin(), yList.end())));

    // Sampling rate
    int sr = sqrt(sample_rate);

    // Holds a sample
    Vector3D s(0, 0, 0);

    // Iterate through each pixel
    for (int x = minX; x <= maxX; x++) {
      for (int y = minY; y <= maxY; y++) {

        // Creating samples based on sampling rate
        int index = 0;
        for (int i = 0; i < sr; i++) {
          s.x = (float) x + ((float) i + 0.5) / sr;
          for (int j = 0; j < sr; j++) {
            s.y = (float) y + ((float) j + 0.5) / sr;
            
            if (inside(p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, s.x, s.y)) {
              fill_supersampled(x, y, index, color);
            }
            index++;
          }
        }
      }
    }
  }
    // TODO: Task 2: Update to implement super-sampled rasterization


  void RasterizerImp::rasterize_interpolated_color_triangle(float x0, float y0, Color c0,
    float x1, float y1, Color c1,
    float x2, float y2, Color c2)
  {
    // TODO: Task 4: Rasterize the triangle, calculating barycentric coordinates and using them to interpolate vertex colors across the triangle
    // Hint: You can reuse code from rasterize_triangle
  
    Vector3D p0(x0, y0, 0);
    Vector3D p1(x1, y1, 0);
    Vector3D p2(x2, y2, 0);

    // Right-hand rule for checking whether winding direction is not ccw
    if (cross(p1 - p0, p2 - p0).z < 0) {
      p1.x = x2;
      p1.y = y2;
      p2.x = x1;
      p2.y = y1;
    }

    // Borders
    vector<float> xList = { x0, x1, x2 };
    vector<float> yList = { y0, y1, y2 };
    int maxX = (ceil(*max_element(xList.begin(), xList.end())));
    int minX = (floor(*min_element(xList.begin(), xList.end())));
    int maxY = (ceil(*max_element(yList.begin(), yList.end())));
    int minY = (floor(*min_element(yList.begin(), yList.end())));

    // Sampling rate
    int sr = sqrt(sample_rate);

    // Holds a sample
    Vector3D s(0, 0, 0);
    
    // P = P1 * u + P2 * v + P3 * w = Matrix P * [u v w]
    // [u v w] = Inverse Matrix P' * P
    Matrix3x3 p(x0, x1, x2, y0, y1, y2, 1, 1, 1);
    Matrix3x3 p_inv = p.inv();
    Vector3D w;

    // Iterate through each pixel
    for (int x = minX; x <= maxX; x++) {
      for (int y = minY; y <= maxY; y++) {

        // Creating samples based on sampling rate
        int index = 0;
        for (int i = 0; i < sr; i++) {
          s.x = (float) x + ((float) i + 0.5) / sr;
          for (int j = 0; j < sr; j++) {
            s.y = (float) y + ((float) j + 0.5) / sr;

            Vector3D p(x, y, 1); // To calculate baycentric coefficients
            
            if (inside(p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, s.x, s.y)) {
              w = p_inv * p;
              w.z = 1 - w.x - w.y;
              fill_supersampled(x, y, index, w.x * c0 + w.y * c1 + w.z * c2);
            }
            
            index++;
          }
        }
      }
    }
  }


  void RasterizerImp::rasterize_textured_triangle(float x0, float y0, float u0, float v0,
    float x1, float y1, float u1, float v1,
    float x2, float y2, float u2, float v2,
    Texture& tex)
  {
    // TODO: Task 5: Fill in the SampleParams struct and pass it to the tex.sample function.
    // TODO: Task 6: Set the correct barycentric differentials in the SampleParams struct.
    // Hint: You can reuse code from rasterize_triangle/rasterize_interpolated_color_triangle

    Vector3D p0(x0, y0, 0);
    Vector3D p1(x1, y1, 0);
    Vector3D p2(x2, y2, 0);

    // Right-hand rule for checking whether winding direction is not ccw
    if (cross(p1 - p0, p2 - p0).z < 0) {
      p1.x = x2;
      p1.y = y2;
      p2.x = x1;
      p2.y = y1;
    }

    // Borders
    vector<float> xList = { x0, x1, x2 };
    vector<float> yList = { y0, y1, y2 };
    int maxX = (ceil(*max_element(xList.begin(), xList.end())));
    int minX = (floor(*min_element(xList.begin(), xList.end())));
    int maxY = (ceil(*max_element(yList.begin(), yList.end())));
    int minY = (floor(*min_element(yList.begin(), yList.end())));

    // Sampling rate
    int sr = sqrt(sample_rate);

    // Holds a sample
    Vector3D s(0, 0, 0);
    
    // P = P1 * u + P2 * v + P3 * w = Matrix P * [u v w]
    // [u v w] = Inverse Matrix P' * P
    Matrix3x3 p(x0, x1, x2, y0, y1, y2, 1, 1, 1);
    Matrix3x3 p_inv = p.inv();
    Vector3D u(u0, u1, u2);
    Vector3D v(v0, v1, v2);

    // Picking sampling method
    SampleParams S;
    S.lsm = lsm;
    S.psm = psm;

    // Iterate through each pixel
    for (int x = minX; x <= maxX; x++) {
      for (int y = minY; y <= maxY; y++) {

        // Creating samples based on sampling rate
        int index = 0;
        for (int i = 0; i < sr; i++) {
          s.x = (float) x + ((float) i + 0.5) / sr;
          for (int j = 0; j < sr; j++) {
            s.y = (float) y + ((float) j + 0.5) / sr;

            Vector3D p(x, y, 1); // To calculate baycentric coefficients
            
            if (inside(p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, s.x, s.y)) {
              // Get barycentric coordinates
              Vector3D b = p_inv * p;
              b.z = 1 - b.x - b.y;
              Vector3D bX = p_inv * (p + Vector3D(1, 0, 0)); // rate of change in x-direction
              bX.z = 1 - bX.x - bX.y;
              Vector3D bY = p_inv * (p + Vector3D(0, 1, 0)); // rate of change in y-direction
              bY.z = 1 - bY.x - bY.y;

              // Convert to texture coordinates
              Vector2D p_uv = Vector2D(dot(b, u), dot(b, v));
              Vector2D p_uv_dx = Vector2D(dot(bX, u), dot(bX, v));
              Vector2D p_uv_dy = Vector2D(dot(bY, u), dot(bY, v));

              S.p_uv = p_uv;
              S.p_dx_uv = p_uv_dx - p_uv;
              S.p_dy_uv = p_uv_dy - p_uv;

              Color c = tex.sample(S);
              fill_supersampled(x, y, index, c);
            }
            index++;
          }
        }
      }
    }
  }

  void RasterizerImp::set_sample_rate(unsigned int rate) {
    // TODO: Task 2: You may want to update this function for supersampling support

    this->sample_rate = rate;


    this->sample_buffer.resize(sample_rate * width * height, Color::White);
  }


  void RasterizerImp::set_framebuffer_target(unsigned char* rgb_framebuffer,
    size_t width, size_t height)
  {
    // TODO: Task 2: You may want to update this function for supersampling support

    this->width = width;
    this->height = height;
    this->rgb_framebuffer_target = rgb_framebuffer;


    this->sample_buffer.resize(sample_rate * width * height, Color::White);
  }


  void RasterizerImp::clear_buffers() {
    std::fill(rgb_framebuffer_target, rgb_framebuffer_target + 3 * width * height, 255);
    std::fill(sample_buffer.begin(), sample_buffer.end(), Color::White);
  }


  // This function is called at the end of rasterizing all elements of the
  // SVG file.  If you use a supersample buffer to rasterize SVG elements
  // for antialising, you could use this call to fill the target framebuffer
  // pixels from the supersample buffer data.
  //
  void RasterizerImp::resolve_to_framebuffer() {
    // TODO: Task 2: You will likely want to update this function for supersampling support


    for (int x = 0; x < width; ++x) {
      for (int y = 0; y < height; ++y) {
        Color col = sample_buffer[sample_rate * (y * width + x)];

        for (int i = 1; i < sample_rate; i++) {
          col += sample_buffer[sample_rate * (y * width + x) + i];
        }
        col *= ((float) 1 / (float) sample_rate);

        for (int k = 0; k < 3; ++k) {
          this->rgb_framebuffer_target[3 * (y * width + x) + k] = (&col.r)[k] * 255;
        }
      }
    }

  }

  Rasterizer::~Rasterizer() { }


}// CGL
