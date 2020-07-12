// Copyright (C) 2014-2020 Russell Smith.
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.

// @@@ TODO: There are static variables here that should really be protected
// with a mutex or made into thread local storage with __thread. Currently this
// code assumes that such things are not required because there is only one
// rendering thread.

#include "gl_utils.h"
#include "shaders.h"
#include "error.h"

#ifdef QT_CORE_LIB
#include "QOpenGLFramebufferObjectFormat"
#endif

using namespace Eigen;

namespace gl {

//***************************************************************************
// Public functions.

#ifdef QT_CORE_LIB

QOpenGLFunctions_3_3_Core gl_functions;

// We want OpenGL 3.3 core profile or later.
const int kGLMajorVersion = 3;
const int kGLMinorVersion = 3;

void SetDefaultOpenGLSurfaceFormat() {
  // From the QOpenGLWidget class documentation:
  // Calling QSurfaceFormat::setDefaultFormat() before constructing the
  // QApplication instance is mandatory on some platforms (for example,
  // macOS) when an OpenGL core profile context is requested. This is to
  // ensure that resource sharing between contexts stays functional as all
  // internal contexts are created using the correct version and profile.
  QSurfaceFormat format;
  format.setDepthBufferSize(32);
  format.setVersion(kGLMajorVersion, kGLMinorVersion);
  format.setProfile(QSurfaceFormat::CoreProfile);
  format.setSamples(4);     // Multisampling (0 to disable)
  QSurfaceFormat::setDefaultFormat(format);
}

void InitializeOpenGLFunctions(QOpenGLContext *context) {
  if (!gl_functions.initializeOpenGLFunctions()) {
    Panic("OpenGL could not be initialized. Desired version is %d.%d, "
          "available version is %d.%d", kGLMajorVersion, kGLMinorVersion,
          context->format().majorVersion(),
          context->format().minorVersion());
  }
}

#endif  // QT_CORE_LIB

GLint CurrentProgram() {
  GLint program = 0;
  GL(GetIntegerv)(GL_CURRENT_PROGRAM, &program);
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
  int transform_loc = GL(GetUniformLocation)(program, "transform");
  if (transform_loc != -1) {
    GL(UniformMatrix4fv)(transform_loc, 1, GL_FALSE, last_T.data());
  }

  int modelview_loc = GL(GetUniformLocation)(program, "modelview");
  if (modelview_loc != -1) {
    GL(UniformMatrix4fv)(modelview_loc, 1, GL_FALSE, last_M.data());
  }

  int normalmatrix_loc = GL(GetUniformLocation)(program, "normalmatrix");
  if (normalmatrix_loc != -1) {
    Matrix3f Nf = last_M.block(0, 0, 3, 3);
    GL(UniformMatrix3fv)(normalmatrix_loc, 1, GL_FALSE, Nf.data());
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
  GL(PixelStorei)(GL_PACK_SWAP_BYTES, 0);
  GL(PixelStorei)(GL_PACK_ROW_LENGTH, 0);
  GL(PixelStorei)(GL_PACK_SKIP_PIXELS, 0);
  GL(PixelStorei)(GL_PACK_SKIP_ROWS, 0);
  GL(PixelStorei)(GL_PACK_ALIGNMENT, 1);

  GL(PixelStorei)(GL_UNPACK_SWAP_BYTES, 0);
  GL(PixelStorei)(GL_UNPACK_ROW_LENGTH, 0);
  GL(PixelStorei)(GL_UNPACK_SKIP_PIXELS, 0);
  GL(PixelStorei)(GL_UNPACK_SKIP_ROWS, 0);
  GL(PixelStorei)(GL_UNPACK_ALIGNMENT, 1);
}

float ReadDepthValue(uint32_t fbo, int x, int y) {
#ifdef QT_CORE_LIB
  // We take an 'fbo' argument here because QOpenGLWidget renders into an
  // underlying FBO that is different from the default. First we try
  // glReadPixels() directly, but if that doesn't do anything then it's likely
  // because we can't read depth components from an FBO if multi sampling is
  // enabled. So instead we use an additional FBO without multisampling to
  // which the depth buffer can be copied. This is not fast, but this function
  // is usually only called for mouse clicks so the speed is not a problem.

  GL(BindFramebuffer)(GL_FRAMEBUFFER, fbo);
  float depth = 1e9;
  GL(ReadPixels)(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
  if (depth >= 0 && depth <= 1) {
    return depth;
  }

  {
    QOpenGLFramebufferObjectFormat format;
    format.setSamples(0);
    format.setAttachment(QOpenGLFramebufferObject::CombinedDepthStencil);
    QOpenGLFramebufferObject fbo2(1, 1, format);
    CHECK(fbo2.isValid());
    GL(BindFramebuffer)(GL_READ_FRAMEBUFFER, fbo);
    GL(BindFramebuffer)(GL_DRAW_FRAMEBUFFER, fbo2.handle());
    GL(BlitFramebuffer)(x, y, x+1, y+1, 0, 0, 1, 1,
        GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT, GL_NEAREST);
    fbo2.bind();
    GL(ReadPixels)(0, 0, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
  }
  // Restore state. The fbo2 destructor will re-bind the current context's
  // default framebuffer but that may not be the same as fbo, so re-bind fbo.
  GL(BindFramebuffer)(GL_FRAMEBUFFER, fbo);
  return depth;
#else
  GL(BindFramebuffer)(GL_FRAMEBUFFER, fbo);
  float depth = 0;  // Must initialize, buggy OpenGLs fail if this is NaN
  GL(ReadPixels)(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
  return depth;
#endif
}

bool PixelToModelCoordinates(uint32_t fbo, int x, int y,
                             const Matrix4d &transform, Vector3d *p) {
  // Find the depth of this pixel.
  float depth = ReadDepthValue(fbo, x, y);
  bool found_good_depth = (depth != 1.0);
  Vector3d xyd(x, y, depth);
  PixelToModelCoordinates(xyd, transform, p);
  return found_good_depth;
}

void PixelToModelCoordinates(const Vector3d &xyd, const Matrix4d &transform,
                             Vector3d *p, Vector3d *dp_by_dd) {
  GLint viewport[4];
  GL(GetIntegerv)(GL_VIEWPORT, viewport);
  UnProject(xyd[0], xyd[1], xyd[2], transform, viewport, p, dp_by_dd);
}

void ModelToPixelCoordinates(const Vector3d &m,
                             const Eigen::Matrix4d &transform, Vector3d *v) {
  GLint viewport[4];
  GL(GetIntegerv)(GL_VIEWPORT, viewport);
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
  GL(GetIntegerv)(GL_VIEWPORT, v);

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
  GL(GenTextures)(1, &tex_);
  GL(ActiveTexture)(GL_TEXTURE0);
  GL(BindTexture)(GL_TEXTURE_2D, tex_);
  GL(TexImage2D)(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0,
                 GL_RGB, GL_UNSIGNED_BYTE, data);
  GL(GenerateMipmap)(GL_TEXTURE_2D);
}

Texture2D::~Texture2D() {
  GL(DeleteTextures)(1, &tex_);
}

//***************************************************************************
// TextureRectangle.

TextureRectangle::TextureRectangle(int width, int height, unsigned char *data) {
  SetNormalPixelPacking();
  GL(GenTextures)(1, &tex_);
  GL(ActiveTexture)(GL_TEXTURE0);
  GL(BindTexture)(GL_TEXTURE_RECTANGLE, tex_);
  GL(TexImage2D)(GL_TEXTURE_RECTANGLE, 0, GL_RGB, width, height, 0,
                 GL_RGB, GL_UNSIGNED_BYTE, data);
}

TextureRectangle::~TextureRectangle() {
  GL(DeleteTextures)(1, &tex_);
}

//***************************************************************************
// Texture3D.

Texture3D::Texture3D(int width, int height, int depth, unsigned char *data,
                     int internal_format, int format) {
  SetNormalPixelPacking();
  GL(GenTextures)(1, &tex_);
  GL(ActiveTexture)(GL_TEXTURE0);
  GL(BindTexture)(GL_TEXTURE_3D, tex_);
  GL(TexImage3D)(GL_TEXTURE_3D, 0, internal_format, width, height, depth, 0,
                 format, GL_UNSIGNED_BYTE, data);
}

Texture3D::~Texture3D() {
  GL(DeleteTextures)(1, &tex_);
}

}  // namespace gl
