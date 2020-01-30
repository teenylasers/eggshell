// Include OpenGL headers and provide various utility functions.

#ifndef __TOOLKIT_GL_UTILS_H__
#define __TOOLKIT_GL_UTILS_H__

#include "error.h"
#include "myvector"
#include <functional>

// Include OpenGL headers.
#ifdef QT_CORE_LIB
  #include <QOpenGLFunctions_3_3_Core>
  #include "undef.h"
#else
  #if defined __APPLE__
    #include <OpenGL/gl3.h>
  #else
    // Linux.
    #define GL_GLEXT_PROTOTYPES
    #include <GL/gl.h>
    #include <GL/glext.h>
  #endif
  #define GL(fn) gl##fn
#endif

#include "Eigen/Dense"

// For pinpointing OpenGL errors sprinkle this macro throughout your code.
#define __SEE_IF_OPENGL_ERROR { \
  int err; \
  while ((err = GL(GetError)()) != GL_NO_ERROR) { \
    printf("GL error %d (%s) at %s:%d\n", err, gl::ErrorString(err), \
           __FILE__, __LINE__); \
    gl::FoundError();   /* Put a breakpoint in this fn to catch errors */ \
  } \
}

namespace gl {

#ifdef QT_CORE_LIB
  extern QOpenGLFunctions_3_3_Core gl_functions;
  #ifdef __WINNT__
    #define GL(fn) gl::gl_functions.gl##fn
  #else
    #define GL(fn) gl##fn
  #endif
  void SetDefaultOpenGLSurfaceFormat();
  void InitializeOpenGLFunctions(QOpenGLContext *context);
#endif

// Return an attribute list that can be passed to the constructor of
// wxGLContext. The 'type' is an OR of the buffer type constants below. This
// function hides various platform bugs in wxWidgets. The returned array is
// valid until the next call to this function.
enum {
  DoubleBuffer      = 1,
  DepthBuffer       = 2,
  AlphaBuffer       = 4,
  MultiSampleBuffer = 8
};
const int *GetAttributeList(int type);

// Return the current program, it's a runtime error if there isn't one.
GLint CurrentProgram();

// Given a projection matrix P and and model-view matrix M, set the following
// uniform shader variables in the currently bound program:
//   * transform = P * M (if "transform" exists).
//   * modelview = M (if "modelview" exists).
//   * normalmatrix = the 3x3 rotational block of M (if "normalmatrix" exists).
void ApplyTransform(const Eigen::Matrix4d &P, const Eigen::Matrix4d &M);

// Call ApplyTransform() with the same arguments that were last used. This is
// convenient when changing shader programs.
void ReapplyTransform();

// Return the last P and M matrices given to ApplyTransform().
Eigen::Matrix4d Projection();
Eigen::Matrix4d ModelView();
Eigen::Matrix4d Transform();

// Compute projection matrices for the given frustums. The width and height
// refer to the near end of the frustum.
Eigen::Matrix4d PerspectiveProjection(double width, double height,
                                      double near, double far);
Eigen::Matrix4d OrthographicProjection(double width, double height,
                                       double near, double far);

// Standard transformation matrices.
Eigen::Matrix4d Translate(Eigen::Vector3d position_xyz);
Eigen::Matrix4d Scale(Eigen::Vector3d scale_xyz);
inline Eigen::Matrix4d Scale(double s) {
  return Scale(Eigen::Vector3d(s, s, s));
}

// Set uniform values in the current program.
inline void SetUniform(const char *name, float value) {
  GLuint loc = GL(GetUniformLocation)(CurrentProgram(), name);
  CHECK(loc != -1);
  GL(Uniform1f)(loc, value);
}
inline void SetUniformi(const char *name, int value) {
  GLuint loc = GL(GetUniformLocation)(CurrentProgram(), name);
  CHECK(loc != -1);
  GL(Uniform1i)(loc, value);
}
inline void SetUniform(const char *name, float v0, float v1) {
  GLuint loc = GL(GetUniformLocation)(CurrentProgram(), name);
  CHECK(loc != -1);
  GL(Uniform2f)(loc, v0, v1);
}
inline void SetUniform(const char *name, float v0, float v1, float v2) {
  GLuint loc = GL(GetUniformLocation)(CurrentProgram(), name);
  CHECK(loc != -1);
  GL(Uniform3f)(loc, v0, v1, v2);
}
inline void SetUniform(const char *name, const Eigen::Vector3d &v) {
  GLuint loc = GL(GetUniformLocation)(CurrentProgram(), name);
  CHECK(loc != -1);
  GL(Uniform3f)(loc, v[0], v[1], v[2]);
}
inline void SetUniform(const char *name, float v0, float v1, float v2,
                       float v3) {
  GLuint loc = GL(GetUniformLocation)(CurrentProgram(), name);
  CHECK(loc != -1);
  GL(Uniform4f)(loc, v0, v1, v2, v3);
}
inline void SetUniform(const char *name, const Eigen::Vector4d &v) {
  GLuint loc = GL(GetUniformLocation)(CurrentProgram(), name);
  CHECK(loc != -1);
  GL(Uniform4f)(loc, v[0], v[1], v[2], v[3]);
}
inline void SetUniform(const char *name, const Eigen::Matrix4f &M) {
  GLuint loc = GL(GetUniformLocation)(CurrentProgram(), name);
  CHECK(loc != -1);
  GL(UniformMatrix4fv)(loc, 1, GL_FALSE, M.data());
}

// Set the "normal" pixel packing modes.
void SetNormalPixelPacking();

// Given pixel coordinates x,y and depth in 'xyd', return the corresponding
// model coordinates given the current camera transform and viewport. The x,y
// are viewport coordinates where 0,0 is the bottom left of the window. If
// dp_by_dd it is set to d(p)/d(xyz[2]), see UnProject().
void PixelToModelCoordinates(const Eigen::Vector3d &xyd,
                             const Eigen::Matrix4d &transform,
                             Eigen::Vector3d *p, Eigen::Vector3d *dp_by_dd = 0);

// A version of PixelToModelCoordinates() where the depth is looked up from the
// currently rendered scene in the depth buffer. Return true if x,y corresponds
// to an actual model pixel, or false if x,y seems to be at the far clip plane.
// If false is returned then 'p' will be the projection of x,y to the far clip
// plane. The result is undefined for pixels outside the window.
bool PixelToModelCoordinates(int x, int y, const Eigen::Matrix4d &transform,
                             Eigen::Vector3d *p);

// Return in v the viewport x,y,depth coordinates of the model coordinate m.
// Note that v is only visible if it is in the view frustum, but this is not
// checked.
void ModelToPixelCoordinates(const Eigen::Vector3d &m,
                             const Eigen::Matrix4d &transform,
                             Eigen::Vector3d *v);

// Draw an axis aligned cube at the given position with the given side lengths.
// Use side colors scaled by 'brightness', or don't use colors at all if
// brightness==0. Use the given alpha value. This is mostly for debugging.
void DrawCube(const Eigen::Vector3d &pos, double side, float brightness,
              float alpha = 1);
void DrawCube(const Eigen::Vector3d &pos, const Eigen::Vector3d &side,
              float brightness, float alpha = 1);
void DrawCubeWireframe(const Eigen::Vector3d &pos,
                       const Eigen::Vector3d &side);

// Return the projection of the bounding box [xmin,xmax,ymin,ymax,zmin,zmax]
// to the unit vector 'v'. The smallest and largest values of the projection
// are return in pmin and pmax.
void ProjectBox(const Eigen::Vector3d &v, const double box[6],
                double *pmin, double *pmax);

// Some GLU-like functionality (since parts of GLU are now deprecated and much
// of it doesn't work with the core profile). The transform matrix is the
// projection * modelview. If dobj_by_dwinz is nonzero it is set to
// d(obj)/d(winz), which is useful when the depth is not known and we want to
// project to a model plane.
void Project(double objx, double objy, double objz,
             const Eigen::Matrix4d &transform, const int viewport[4],
             Eigen::Vector3d *win);
void UnProject(double winx, double winy, double winz,
               const Eigen::Matrix4d &transform, const int viewport[4],
               Eigen::Vector3d *obj, Eigen::Vector3d *dobj_by_dwinz = 0);
void PickMatrix(GLdouble x, GLdouble y, GLdouble deltax, GLdouble deltay,
                GLint viewport[4]);
const char *ErrorString(int error);

// A dummy function to put a breakpoint on when doing GL error finding.
void FoundError();

// Utility to draw thick lines, a feature taken out in GL core. The do_drawing
// function draws one copy of the scene. The scene will be redrawn x_steps
// times, each time shifting everything horizontally by one pixel (similarly
// for y_steps). If full is true then x_steps*y_steps drawings will be done,
// i.e. with a "square brush". Otherwise x_steps+y_steps-1 drawings will be
// done, i.e. with a "plus-shaped "brush".
void DrawThick(int x_steps, int y_steps, bool full,
               std::function<void()> do_drawing);

// Vertex array objects (VAOs) store all of the state needed to supply vertex
// data. This can be used as a static variable to create a VAO once. See
// https://www.opengl.org/wiki/Vertex_Specification
class VertexArray {
 public:
  VertexArray() : ok_(false), vao_(0) {}
  ~VertexArray() { if (ok_) GL(DeleteVertexArrays)(1, &vao_); }
  void Bind() {
    if (!ok_) GL(GenVertexArrays)(1, &vao_);
    ok_ = true;
    GL(BindVertexArray)(vao_);
  }
  bool Initialized() const { return ok_; }
 private:
  bool ok_;
  GLuint vao_;
};

// Buffer for vertices, colors etc. This can be used as a static variable to
// create a buffer from static data once.
class Buffer {
 public:
  // If data2==0 then size2 will be assumed to be zero.
  Buffer(int size1, const void *data1, int size2 = 0, const void *data2 = 0,
         int kind = GL_ARRAY_BUFFER, int usage = GL_STATIC_DRAW)
      : ok_(false), size1_(size1), size2_(size2), kind_(kind), usage_(usage),
        data1_(data1), data2_(data2), buf_(0) {}
  ~Buffer() { if (ok_) GL(DeleteBuffers)(1, &buf_); }
  void Bind() {
    if (!ok_) {
      GL(GenBuffers)(1, &buf_);
      ok_ = true;
      GL(BindBuffer)(kind_, buf_);
      GL(BufferData)(kind_, size1_ + (data2_ ? size2_ : 0), 0, usage_);
      GL(BufferSubData)(kind_, 0, size1_, data1_);
      if (data2_) {
        GL(BufferSubData)(kind_, size1_, size2_, data2_);
      }
    } else {
      GL(BindBuffer)(kind_, buf_);
    }
  }
  bool Initialized() const { return ok_; }
 private:
  bool ok_;
  int size1_, size2_, kind_, usage_;
  const void *data1_, *data2_;
  GLuint buf_;
};

// A specialization of Buffer that stores element indexes for drawing.
class IndexBuffer : public Buffer {
 public:
  IndexBuffer(int index_count, const GLint *data)
    : Buffer(index_count * sizeof(GLint), data, 0, 0, GL_ELEMENT_ARRAY_BUFFER),
      index_count_(index_count) {}
  int IndexCount() const { return index_count_; }
 private:
  int index_count_;
};

// Allow specification of vertex attributes within a buffer, and drawing
// geometry from the buffer. Class instances can be used as static variables to
// create a vertex buffer from static data just once. T1, T2 etc are assumed to
// each contain some combination of all vertex information (position, color,
// normals etc), the buffer will be an array of T1 followed by an array of T2,
// etc.
template<class T1, class T2 = float>
class VertexBuffer {
 public:
  // The buffer will store one "T1" and one "T2" for every vertex. If data2==0
  // then T2 is considered to be zero sized.
  VertexBuffer(int vertex_count, const T1 *data1, const T2 *data2 = 0)
      : buffer_(vertex_count*sizeof(T1), data1, vertex_count*sizeof(T2), data2),
        vertex_count_(vertex_count) {}
  VertexBuffer(const std::vector<T1> &v)
      : buffer_(v.size()*sizeof(T1), v.data()), vertex_count_(v.size()) {}
  VertexBuffer(const std::vector<T1> &v1, const std::vector<T2> &v2)
      : buffer_(v1.size()*sizeof(T1), v1.data(),
                v2.size()*sizeof(T2), v2.data()), vertex_count_(v1.size())
      { CHECK(v1.size() == v2.size()); }
  bool Initialized() const { return vao_.Initialized(); }
  // Specify<n> specifies the layout of T<n> to opengl. The name is a shader program
  // attribute, T_offset is the byte offset into T<n>, where there are "count"
  // entities of the given type (e.g. type==GL_FLOAT).
  void Specify1(const char *name, int T_offset, int count, int type) {
    Specify(name, count, type, sizeof(T1), T_offset);
  }
  void Specify2(const char *name, int T_offset, int count, int type) {
    Specify(name, count, type, sizeof(T2), vertex_count_*sizeof(T1) + T_offset);
  }
  // Emit geometry in the buffer. The mode is e.g. GL_TRIANGLE_STRIP. The start
  // and count indicate which vertices to emit, the default is all vertices.
  void Draw(int mode, int start = 0, int count = -1) {
    vao_.Bind();
    GL(DrawArrays)(mode, start, count >= 0 ? count : vertex_count_);
  }
  // Emit geometry in the buffer via indexes contained in the IndexBuffer.
  void DrawIndexed(int mode, IndexBuffer *index_buffer) {
    vao_.Bind();
    index_buffer->Bind();
    GL(DrawElements)(mode, index_buffer->IndexCount(), GL_UNSIGNED_INT, 0);
  }
 private:
  VertexArray vao_;
  Buffer buffer_;
  int vertex_count_;
  void Specify(const char *name, int count, int type, int stride, int offset) {
    vao_.Bind();
    buffer_.Bind();
    GLint loc = GL(GetAttribLocation)(CurrentProgram(), name);
    CHECK(loc != -1);
    GL(VertexAttribPointer)(loc, count, type, GL_FALSE, stride,
                            ((char*)0) + offset);
    GL(EnableVertexAttribArray)(loc);
  }
};

// Utility for drawing a vector<Vector3f> of vertex coordinates.
inline void Draw(const std::vector<Eigen::Vector3f> &v, int mode) {
  gl::VertexBuffer<Eigen::Vector3f> buffer(v);
  buffer.Specify1("vertex", 0, 3, GL_FLOAT);
  buffer.Draw(mode);
}

// Utility for drawing two vector<Vector3f> of vertex coordinates and colors.
inline void Draw(const std::vector<Eigen::Vector3f> &v,
                 const std::vector<Eigen::Vector3f> &c, int mode) {
  gl::VertexBuffer<Eigen::Vector3f, Eigen::Vector3f> buffer(v, c);
  buffer.Specify1("vertex", 0, 3, GL_FLOAT);
  buffer.Specify2("vertex_color", 0, 3, GL_FLOAT);
  buffer.Draw(mode);
}

// A 2D texture.
class Texture2D {
 public:
  Texture2D(int width, int height, unsigned char *data);
  ~Texture2D();
  void Bind() const { GL(BindTexture)(GL_TEXTURE_2D, tex_); }

 private:
  GLuint tex_;
};

// A rectangle texture.
class TextureRectangle {
 public:
  TextureRectangle(int width, int height, unsigned char *data);
  ~TextureRectangle();
  void Bind() const { GL(BindTexture)(GL_TEXTURE_RECTANGLE, tex_); }

 private:
  GLuint tex_;
};

// A 3D texture.
class Texture3D {
 public:
  Texture3D(int width, int height, int depth, unsigned char *data,
            int internal_format = GL_RGB, int format = GL_RGB);
  ~Texture3D();
  void Bind() const { GL(BindTexture)(GL_TEXTURE_3D, tex_); }

 private:
  GLuint tex_;
};

}  // namespace gl

#endif
