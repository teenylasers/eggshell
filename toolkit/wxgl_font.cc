
#include "stdwx.h"
#include "wxgl_font.h"
#include "error.h"
#include "gl_utils.h"
#include "shaders.h"
#include <stdint.h>

void DrawString(const char *sarg, double x, double y, const wxFont &font,
                TextAlignment halign, TextAlignment valign) {
  const uint8_t *s = (const uint8_t *) sarg;
  if (!s || !s[0]) {
    return;
  }

  // Create the memory DC and measure the text.
  wxMemoryDC memdc;
  memdc.SetFont(font);
  wxCoord w, h, descent;
  memdc.GetTextExtent(s, &w, &h, &descent);
  AlignText(w, h, descent, halign, valign, false, &x, &y);

  // Create the bitmap to render into and set text rendering mode.
  wxBitmap bitmap(w, h);                //@@@ cache bitmaps or DCs?
  memdc.SelectObject(bitmap);
  memdc.SetTextBackground(*wxWHITE);
  memdc.SetTextForeground(*wxBLACK);
  memdc.SetBackgroundMode(wxPENSTYLE_SOLID);
  memdc.DrawText(s, 0, 0);
  memdc.SelectObject(wxNullBitmap);     // Now bitmap data can be used
  memdc.SetFont(wxNullFont);

  // Shader with pixel coordinate mapping.
  static gl::Shader shader(
    // Vertex shader.
    " #version 330 core\n"
    " uniform float vwidth, vheight; "  // Viewport width and height
    " uniform float x, y; "             // Screen coordinates of image TL
    " in vec2 vertex;"
    " out vec2 tex_coord;"
    " void main() {"
    "   gl_Position = vec4((x + vertex.x) * 2.0 / vwidth - 1.0,"
    "                      (y - vertex.y) * 2.0 / vheight - 1.0, 0, 1);"
    "   tex_coord = vertex.xy;"
    " }",
    // Fragment shader.
    " #version 330 core\n"
    " uniform sampler2DRect tex;"
    " in vec2 tex_coord;"
    " out vec4 frag_color;"
    " void main() {"
    "   frag_color = texture(tex, tex_coord);"
    " }");
  gl::PushShader push_shader(shader);
  GLint V[4];                           // viewport x,y,width,height
  glGetIntegerv(GL_VIEWPORT, V);
  gl::SetUniform("vwidth", V[2]);
  gl::SetUniform("vheight", V[3]);
  gl::SetUniform("x", x);
  gl::SetUniform("y", y);

  // Render the image.
  wxImage image = bitmap.ConvertToImage();
  gl::TextureRectangle tex(image.GetWidth(), image.GetHeight(),
                           image.GetData());
  glBlendFunc(GL_ZERO, GL_SRC_COLOR);   // Blending mode ensures text background
  glEnable(GL_BLEND);                   //   is transparent
  tex.Bind();
  float iw = image.GetWidth(), ih = image.GetHeight();
  float xy[][2] = {{0, 0}, {iw, 0}, {0, ih}, {iw, ih}};
  gl::VertexBuffer<float[2]> buffer(4, xy);
  buffer.Specify1("vertex", 0, 2, GL_FLOAT);
  buffer.Draw(GL_TRIANGLE_STRIP);
  glDisable(GL_BLEND);
}

void DrawStringM(const char *s, double x, double y, double z,
                 const Eigen::Matrix4d &T, const wxFont &font,
                 TextAlignment halign, TextAlignment valign) {
  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);
  Eigen::Vector3d win;
  gl::Project(x, y, z, T, viewport, &win);
  DrawString(s, win[0], win[1], font, halign, valign);
}
