
// @@@ TODO: There are static variables here that should really be protected
// with a mutex or made into thread local storage with __thread. Currently this
// code assumes that such things are not required because there is only one
// rendering thread.

#ifdef __TOOLKIT_WXWINDOWS__
#include "stdwx.h"
#endif

#include "gl_utils.h"
#include "shaders.h"
#include "error.h"

using namespace Eigen;

namespace gl {

//***************************************************************************
// Windows support for some OpenGL >v1.1 functions.

// Get the address of a GL entry point. This is needed on windows, where the
// GL static libraries only supply functions up to OpenGL 1.1 but the
// underlying DLLs support much more recent OpenGL versions.

#ifdef __WXMSW__

static void *GetGLFunctionAddress(const char *name) {
  // See https://www.opengl.org/wiki/Load_OpenGL_Functions
  void *p = (void *) wglGetProcAddress(name);
  if(p == 0 ||
     (p == (void*)0x1) || (p == (void*)0x2) || (p == (void*)0x3) ||
     (p == (void*)-1) ) {
    // wglGetProcAddress() failed, try a different strategy:
    HMODULE module = LoadLibraryA("opengl32.dll");
    p = (void *)GetProcAddress(module, name);
  }
  CHECK(p);
  return p;
}

#define SUPPORT(name, proto, arguments) \
  void my ## name proto { \
    typedef void (*fn) proto; \
    static void *entry = 0; \
    if (!entry) { \
      entry = GetGLFunctionAddress(#name); \
    } \
    (*(fn)entry)arguments; \
  }

#else

#define SUPPORT(name, proto, arguments) \
  void my ## name proto { \
    name arguments; \
  }

#endif

//***************************************************************************
// Public functions.

#ifdef __TOOLKIT_WXWINDOWS__
const int *GetAttributeList(int type) {
  const int kMaxAttrib = 100;
  static int attrib[kMaxAttrib];
  int i = 0;

  // GL attributes.
  #ifdef __APPLE__
    // The OS X implementation of glcanvas.mm:WXGLChoosePixelFormat() does not
    // parse many WX_GL attributes correctly in wxWidgets 3.0.2 and later.
    // Generate an attribute list that can be parsed correctly.
    #if wxMAJOR_VERSION != 3 || wxMINOR_VERSION != 1 || wxRELEASE_NUMBER != 0
    #error See if this version of wxWidgets can parse attrib_list correctly
    #endif
    if (type & DoubleBuffer) {
      attrib[i++] = WX_GL_DOUBLEBUFFER;
    }
    attrib[i++] = WX_GL_MIN_RED;
    attrib[i++] = 8;
    attrib[i++] = WX_GL_MIN_ALPHA;
    attrib[i++] = (type & AlphaBuffer) ? 8 : 0;
    if (type & DepthBuffer) {
      attrib[i++] = WX_GL_DEPTH_SIZE;
      attrib[i++] = 8;          // 32 e.g. does not work
    }
    // Get at least OpenGL 3.2. On OS X 10.11.4 this gets OpenGL version. 4.1.
    attrib[i++] = WX_GL_CORE_PROFILE;
  #else
    // Windows and linux.
    attrib[i++] = WX_GL_RGBA;
    attrib[i++] = WX_GL_LEVEL;
    attrib[i++] = 0;
    attrib[i++] = WX_GL_MIN_RED;
    attrib[i++] = 8;
    attrib[i++] = WX_GL_MIN_GREEN;
    attrib[i++] = 8;
    attrib[i++] = WX_GL_MIN_BLUE;
    attrib[i++] = 8;
    if (type & AlphaBuffer) {
      attrib[i++] = WX_GL_MIN_ALPHA;
      attrib[i++] = 8;
    }
    if (type & DepthBuffer) {
      attrib[i++] = WX_GL_DEPTH_SIZE;
      attrib[i++] = 32;
    }
    if (type & DoubleBuffer) {
      attrib[i++] = WX_GL_DOUBLEBUFFER;
    }
  #endif
  if (type & MultiSampleBuffer) {
    attrib[i++] = WX_GL_SAMPLE_BUFFERS;   // Multi-sampling
    attrib[i++] = GL_TRUE;
    attrib[i++] = WX_GL_SAMPLES;          // 2x2 antialiasing supersampling
    attrib[i++] = 4;
    attrib[i++] = 0;                      // Terminate the list
    attrib[i++] = 0;
    CHECK(i < kMaxAttrib);

    // On windows IsDisplaySupported() does not properly check for multisample
    // capability but the wxGLCanvas::Create appears to do something
    // intelligent if multisample is not available. On linux this works.
    #ifndef __WXMSW__
      if (wxGLCanvasBase::IsDisplaySupported(attrib)) {
        return attrib;
      }
      // Back out the sample buffer attributes.
      i -= 6;
    #endif
  }
  attrib[i++] = 0;                      // Terminate the list
  attrib[i++] = 0;
  CHECK(i < kMaxAttrib);
  return attrib;
}
#endif  // __TOOLKIT_WXWINDOWS__

GLint CurrentProgram() {
  GLint program = 0;
  glGetIntegerv(GL_CURRENT_PROGRAM, &program);
  CHECK(program);
  return program;
}

static Matrix4f last_P, last_M, last_T;
static struct InitializeMatrices {
  InitializeMatrices() {
    last_P.setZero();
    last_M.setZero();
    last_T.setZero();
  }
} initialize_matrices;

void ApplyTransform(const Eigen::Matrix4d &P, const Eigen::Matrix4d &M) {
  last_P = P.cast<float>();
  last_M = M.cast<float>();
  last_T = last_P * last_M;
  ReapplyTransform();
}

void ReapplyTransform() {
  GLint program = CurrentProgram();
  int transform_loc = glGetUniformLocation(program, "transform");
  if (transform_loc != -1) {
    glUniformMatrix4fv(transform_loc, 1, GL_FALSE, last_T.data());
  }

  int modelview_loc = glGetUniformLocation(program, "modelview");
  if (modelview_loc != -1) {
    glUniformMatrix4fv(modelview_loc, 1, GL_FALSE, last_M.data());
  }

  int normalmatrix_loc = glGetUniformLocation(program, "normalmatrix");
  if (normalmatrix_loc != -1) {
    Matrix3f Nf = last_M.block(0, 0, 3, 3);
    glUniformMatrix3fv(normalmatrix_loc, 1, GL_FALSE, Nf.data());
  }
}

Matrix4d Projection() {
  return last_P.cast<double>();
}

Matrix4d ModelView() {
  return last_M.cast<double>();
}

Matrix4d Transform() {
  return last_T.cast<double>();
}

Matrix4d PerspectiveProjection(double width, double height,
                               double near, double far) {
  Matrix4d M;
  M << 2 * near / width, 0, 0, 0,
       0, 2 * near / height, 0, 0,
       0, 0, (near + far) / (near - far), -2 * far * near / (far - near),
       0, 0, -1, 0;
  return M;
}

Matrix4d OrthographicProjection(double width, double height,
                                double near, double far) {
  Matrix4d M;
  M << 2 / width, 0, 0, 0,
       0, 2 / height, 0, 0,
       0, 0, 2 / (near - far), (near + far) / (near - far),
       0, 0, 0, 1;
  return M;
}

Matrix4d Translate(Vector3d position_xyz) {
  Matrix4d M;
  M << 1, 0, 0, position_xyz[0],
       0, 1, 0, position_xyz[1],
       0, 0, 1, position_xyz[2],
       0, 0, 0, 1;
  return M;
}

Matrix4d Scale(Vector3d scale_xyz) {
  Matrix4d M;
  M << scale_xyz[0], 0, 0, 0,
       0, scale_xyz[1], 0, 0,
       0, 0, scale_xyz[2], 0,
       0, 0, 0, 1;
  return M;
}

void SetNormalPixelPacking() {
  glPixelStorei(GL_PACK_SWAP_BYTES, 0);
  glPixelStorei(GL_PACK_ROW_LENGTH, 0);
  glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
  glPixelStorei(GL_PACK_SKIP_ROWS, 0);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);

  glPixelStorei(GL_UNPACK_SWAP_BYTES, 0);
  glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
  glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
  glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
}

bool PixelToModelCoordinates(int x, int y, const Matrix4d &transform,
                             Vector3d *p) {
  // Find the depth of this pixel. Note that glReadPixels() returns undefined
  // values for pixels outside the window.
  float depth;
  glReadPixels(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
  bool found_good_depth = (depth != 1.0);
  Vector3d xyd(x, y, depth);
  PixelToModelCoordinates(xyd, transform, p);
  return found_good_depth;
}

void PixelToModelCoordinates(const Vector3d &xyd, const Matrix4d &transform,
                             Vector3d *p, Vector3d *dp_by_dd) {
  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);
  UnProject(xyd[0], xyd[1], xyd[2], transform, viewport, p, dp_by_dd);
}

void ModelToPixelCoordinates(const Vector3d &m,
                             const Eigen::Matrix4d &transform, Vector3d *v) {
  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);
  Project(m[0], m[1], m[2], transform, viewport, v);
}

void DrawCube(const Vector3d &pos, double side, float brightness, float alpha) {
  DrawCube(pos, Vector3d(side, side, side), brightness, alpha);
}

void DrawCube(const Vector3d &pos, const Vector3d &side, float brightness,
              float alpha) {
  float x1 = pos[0] - side[0] / 2.0;
  float x2 = pos[0] + side[0] / 2.0;
  float y1 = pos[1] - side[1] / 2.0;
  float y2 = pos[1] + side[1] / 2.0;
  float z1 = pos[2] - side[2] / 2.0;
  float z2 = pos[2] + side[2] / 2.0;

  float b = brightness, a = alpha;
  float xyzrgb[][7] = {
    {x2, y2, z2, b,0,0,a},  {x1, y2, z2, b,0,0,a},  {x2, y1, z2, b,0,0,a},
    {x1, y1, z2, b,0,0,a},  {x2, y1, z2, b,0,0,a},  {x1, y2, z2, b,0,0,a},
    {x2, y1, z2, 0,b,0,a},  {x1, y1, z2, 0,b,0,a},  {x2, y1, z1, 0,b,0,a},
    {x2, y1, z1, 0,b,0,a},  {x1, y1, z2, 0,b,0,a},  {x1, y1, z1, 0,b,0,a},
    {x1, y1, z1, 0,0,b,a},  {x1, y1, z2, 0,0,b,a},  {x1, y2, z1, 0,0,b,a},
    {x1, y2, z1, 0,0,b,a},  {x1, y1, z2, 0,0,b,a},  {x1, y2, z2, 0,0,b,a},
    {x1, y2, z1, b,b,0,a},  {x2, y2, z1, b,b,0,a},  {x2, y1, z1, b,b,0,a},
    {x1, y2, z1, b,b,0,a},  {x2, y1, z1, b,b,0,a},  {x1, y1, z1, b,b,0,a},
    {x1, y2, z2, 0,b,b,a},  {x2, y2, z2, 0,b,b,a},  {x2, y2, z1, 0,b,b,a},
    {x1, y2, z2, 0,b,b,a},  {x2, y2, z1, 0,b,b,a},  {x1, y2, z1, 0,b,b,a},
    {x2, y1, z2, b,0,b,a},  {x2, y1, z1, b,0,b,a},  {x2, y2, z1, b,0,b,a},
    {x2, y1, z2, b,0,b,a},  {x2, y2, z1, b,0,b,a},  {x2, y2, z2, b,0,b,a},
  };

  VertexBuffer<float[7]> buffer(3*12, xyzrgb);
  buffer.Specify1("vertex", 0, 3, GL_FLOAT);
  if (brightness > 0) {
    buffer.Specify1("vertex_color", 3 * sizeof(float), 4, GL_FLOAT);
  }
  buffer.Draw(GL_TRIANGLES);
}

void DrawCubeWireframe(const Vector3d &pos, const Vector3d &side) {
  float x1 = pos[0] - side[0] / 2.0;
  float x2 = pos[0] + side[0] / 2.0;
  float y1 = pos[1] - side[1] / 2.0;
  float y2 = pos[1] + side[1] / 2.0;
  float z1 = pos[2] - side[2] / 2.0;
  float z2 = pos[2] + side[2] / 2.0;
  float xyz[][3] = {
    {x1, y1, z1}, {x1, y1, z2}, {x1, y2, z1}, {x1, y2, z2},
    {x2, y1, z1}, {x2, y1, z2}, {x2, y2, z1}, {x2, y2, z2},
    {x1, y1, z1}, {x1, y2, z1}, {x1, y1, z2}, {x1, y2, z2},
    {x2, y1, z1}, {x2, y2, z1}, {x2, y1, z2}, {x2, y2, z2},
    {x1, y1, z1}, {x2, y1, z1}, {x1, y1, z2}, {x2, y1, z2},
    {x1, y2, z1}, {x2, y2, z1}, {x1, y2, z2}, {x2, y2, z2},
  };
  VertexBuffer<float[3]> buffer(2*12, xyz);
  buffer.Specify1("vertex", 0, 3, GL_FLOAT);
  buffer.Draw(GL_LINES);
}

void ProjectBox(const Vector3d &v, const double box[6],
                double *pmin, double *pmax) {
  Vector3d center((box[0] + box[1]) / 2.0,
                  (box[2] + box[3]) / 2.0,
                  (box[4] + box[5]) / 2.0);
  Vector3d side(box[1] - box[0], box[3] - box[2], box[5] - box[4]);
  double c = center.dot(v);
  double p = 0.5*(fabs(side[0]*v[0]) + fabs(side[1]*v[1]) + fabs(side[2]*v[2]));
  *pmin = c - p;
  *pmax = c + p;
}

void Project(double objx, double objy, double objz,
             const Matrix4d &transform, const int viewport[4], Vector3d *win) {
  Vector4d v = transform * Vector4d(objx, objy, objz, 1);
  v.head(3) /= v[3];
  // Map to range 0..1.
  v.head(3) = v.head(3) * 0.5 + Vector3d(0.5, 0.5, 0.5);
  // Map x,y to viewport.
  v[0] = v[0] * viewport[2] + viewport[0];
  v[1] = v[1] * viewport[3] + viewport[1];
  *win = v.head(3);
}

void UnProject(double winx, double winy, double winz,
               const Matrix4d &transform, const int viewport[4],
               Vector3d *obj, Vector3d *dobj_by_dwinz) {
  Matrix4d final = transform.inverse();

  // Map x and y from window coordinates, then map to range -1..1.
  Vector4d win(winx, winy, winz, 1);
  win[0] = (win[0] - viewport[0]) / viewport[2];
  win[1] = (win[1] - viewport[1]) / viewport[3];
  win.head(3) = win.head(3) * 2.0 - Vector3d(1, 1, 1);

  // Compute final result.
  Vector4d result = final * win;
  *obj = result.head(3) / result[3];
  if (dobj_by_dwinz) {
    *dobj_by_dwinz = final.col(2).head(3) * 2.0 / result[3];
  }
}

const char *ErrorString(int error) {
  switch (error) {
    case GL_NO_ERROR: return "no error";
    case GL_INVALID_ENUM: return "invalid enumerant";
    case GL_INVALID_VALUE: return "invalid value";
    case GL_INVALID_OPERATION: return "invalid operation";
    case GL_OUT_OF_MEMORY: return "out of memory";
  }
  return "unknown error";
}

void FoundError() {
  // Called by __SEE_IF_OPENGL_ERROR in case you want to set a debugger
  // breakpoint here.
}

void DrawThick(int x_steps, int y_steps, bool full,
               std::function<void()> do_drawing) {
  // Save state
  Eigen::Matrix4d P = gl::Projection(), MV = gl::ModelView();
  int v[4];
  glGetIntegerv(GL_VIEWPORT, v);

  // Draw the scene repeatedly, each time adjusting the viewport and projection
  // matrix to shift everything horizontally / vertically by some number of
  // pixels.
  for (int xofs = -x_steps/2; xofs < x_steps - x_steps/2; xofs++) {
    for (int yofs = -y_steps/2; yofs < y_steps - y_steps/2; yofs++) {
      if (full || xofs == 0 || yofs == 0) {
        gl::ApplyTransform(gl::Translate(Eigen::Vector3d(2.0 * xofs / v[2],
                                            2.0 * yofs / v[3], 0)) * P, MV);
        do_drawing();
      }
    }
  }

  // Restore state.
  gl::ApplyTransform(P, MV);
}

//***************************************************************************
// Texture2D.

Texture2D::Texture2D(int width, int height, unsigned char *data) {
  SetNormalPixelPacking();
  glGenTextures(1, &tex_);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, tex_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0,
               GL_RGB, GL_UNSIGNED_BYTE, data);
  glGenerateMipmap(GL_TEXTURE_2D);
}

Texture2D::~Texture2D() {
  glDeleteTextures(1, &tex_);
}

//***************************************************************************
// TextureRectangle.

TextureRectangle::TextureRectangle(int width, int height, unsigned char *data) {
  SetNormalPixelPacking();
  glGenTextures(1, &tex_);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_RECTANGLE, tex_);
  glTexImage2D(GL_TEXTURE_RECTANGLE, 0, GL_RGB, width, height, 0,
               GL_RGB, GL_UNSIGNED_BYTE, data);
}

TextureRectangle::~TextureRectangle() {
  glDeleteTextures(1, &tex_);
}

}  // namespace gl
