
// We assume that we have an RGB frame buffer of at least 8 bit depth, giving
// 2^24 distinct selectable objects. We used to have logic here to handle less
// deep frame buffers that would occur in windows 16 bit displays, but it is
// rare now to have anything smaller that RGB8.
//
// @@@ TODO: Increase number of selectable objects by using alpha channel too.
// @@@ TODO: Need to check that there are at least 8 bits of RGB depth in the
//           back buffer.

#include "gl_utils.h"
#include "color_based_selection.h"

using namespace Eigen;

namespace gl {

// The strategy: render selection-colored objects into the back buffer, then
// read a pixel area around x,y to see what the colors are. In debug mode we
// render to the front buffer to illustrate that things are working.
static const bool kDebug = false;

// The "test box" is a kSize*kSize area around x,y to test pixels of. kSize
// must be odd.
static const int kSize = 11;            // @@@ Multiply by retina scale

// Buffer to draw to and read from.
static const GLenum kSelectionBuffer = kDebug ? GL_FRONT : GL_BACK;

bool ColorBasedSelection::SetColorForSelection(int n) const {
  n++;                          // Black is reserved for the background color
  if (n < 1 || n >= 0x1000000) {
    return false;
  }
  int ri = n & 0xff;
  n >>= 8;
  int gi = n & 0xff;
  n >>= 8;
  int bi = n & 0xff;
  float r = float(ri) / 255.0f;
  float g = float(gi) / 255.0f;
  float b = float(bi) / 255.0f;
  SetUniform("color", r, g, b);
  return true;
}

int ColorBasedSelection::GetSelectionFromColor(unsigned char col[3]) {
  return (((col[2] << 8) + col[1]) << 8) + col[0] - 1;
}

ColorBasedSelection::ColorBasedSelection(int x, int y)
        : push_shader_(gl::FlatShader()) {
  pick_matrix_.setZero();

  // Save the arguments and current viewport size.
  x_ = x;
  y_ = y;
  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);
  width_ = viewport[2];
  height_ = viewport[3];

  // Set the selection rendering state. Disable some things that mess up color
  // measurements.
  glDisable(GL_MULTISAMPLE);
  glDisable(GL_DITHER);
  glDrawBuffer(kSelectionBuffer);

  // Restrict view frustum for more efficient rendering, so that most
  // primitives will be culled by viewport clipping before rasterization.
  int x0 = x - kSize/2;
  int y0 = y - kSize/2;
  glViewport(x0, y0, kSize, kSize);

  // Clear the test box area of the viewport. Don't use glClear() as that would
  // unnecessarily clear the entire viewport.
  if (true) {
    // Use a simple orthographic projection with an identity model-view matrix
    // (i.e. a camera looking at -z).
    ApplyTransform(OrthographicProjection(2, 2, -1, 1), Matrix4d::Identity());

    // Save depth settings.
    GLint saved_depth_func;
    GLfloat saved_depth_range[2];
    glGetIntegerv(GL_DEPTH_FUNC, &saved_depth_func);
    glGetFloatv(GL_DEPTH_RANGE, saved_depth_range);

    // Draw the clearing polygon.
    glDepthFunc(GL_ALWAYS);       // To overwrite existing depth info
    glDepthRange(1, 1);           // To write maximum depth
    if (kDebug) {
      SetUniform("color", 0.5, 0.5, 0.5);       // Visible but incorrect
    } else {
      SetUniform("color", 0, 0, 0);             // Clear color = black
    }

    // Draw a clearing polgon.
    static float xyz[][3] = {{-2,-2,-1}, {2,-2,-1}, {-2,2,-1}, {2,2,-1}};
    static VertexBuffer<float[3]> buffer(3*4, xyz);
    if (!buffer.Initialized()) {
      buffer.Specify1("vertex", 0, 3, GL_FLOAT);
    }
    buffer.Draw(GL_TRIANGLE_STRIP);

    // Restore depth settings.
    glDepthFunc(saved_depth_func);
    glDepthRange(saved_depth_range[0], saved_depth_range[1]);
  } else {
    // The more expensive way, for debugging. We clear to grey rather than the
    // more correct black for easier visualization.
    glClearColor(0.5, 0.5, 0.5, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  }

  // Set a pick matrix that adjusts the projection so that the scene remains on
  // the same pixels in the new viewport.
  double size = kSize;
  pick_matrix_ =
    Translate(Vector3d((width_  - 2 * (x + 0.5 - viewport[0])) / size,
                       (height_ - 2 * (y + 0.5 - viewport[1])) / size, 0)) *
    Scale(Vector3d(width_ / size, height_ / size, 1));
}

int ColorBasedSelection::GetSelection() {
  // Read back the pixels we just drew.
  glFlush();
  glReadBuffer(kSelectionBuffer);
  uint8_t data[kSize*kSize*3];
  SetNormalPixelPacking();
  int x0 = x_ - kSize/2;
  int y0 = y_ - kSize/2;
  glReadPixels(x0, y0, kSize, kSize, GL_RGB,GL_UNSIGNED_BYTE, data);

  // For debugging show the pixels we just read back.
  if (false) {
    for (int i = 0; i < kSize; i++) {
      for (int j = 0; j < 3*kSize; j++) {
        printf("%2x", data[i*kSize*3 + j]);
      }
      printf("\n");
    }
    printf("\n");
    fflush(stdout);
  }

  // Starting from the selection point in the bitmap and working outwards, find
  // the color of the first non-black pixel. Ignore pixels outside the
  // viewport, since glReadPixels() returns undefined values for those.
  for (int r = 0; r <= kSize/2; r++) {                  // r is the 'radius'
    for (int tx = x_ - r; tx <= x_ + r; tx++) {         // test x coordinate
      for (int ty = y_ - r; ty <= y_ + r; ty++) {       // test y coordinate
        if (tx >= 0 && ty >= 0 && tx < width_ && ty < height_) {
          uint8_t *pixel = data + ((ty - y0)*kSize + (tx - x0))*3;
          if (pixel[0] | pixel[1] | pixel[2]) {
            int index = GetSelectionFromColor(pixel);
            CHECK(index >= 0);
            return index;
          }
        }
      }
    }
  }
  return -1;
}

}  // namespace gl
