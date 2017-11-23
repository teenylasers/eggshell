
#define GL_FONT_IMPLEMENTATION

#include "gl_font.h"
#include "error.h"
#include "gl_utils.h"
#include "shaders.h"
#include "myvector"
#include <stdint.h>

struct StringToDraw {
  std::string s;
  const Font *font;
  double x, y;
  float red, green, blue;
  TextAlignment halign, valign;
};

static std::vector<StringToDraw> strings;

void DrawString(const char *s, double x, double y, const Font *font,
                float red, float green, float blue,
                TextAlignment halign, TextAlignment valign) {
  if (!s || !s[0]) {
    return;
  }
  strings.resize(strings.size() + 1);
  strings.back().s = s;
  strings.back().font = font;
  strings.back().x = x;
  strings.back().y = y;
  strings.back().red = red;
  strings.back().green = green;
  strings.back().blue = blue;
  strings.back().halign = halign;
  strings.back().valign = valign;
}

void DrawStringM(const char *s, double x, double y, double z,
                 const Eigen::Matrix4d &T, const Font *font,
                 float red, float green, float blue,
                 TextAlignment halign, TextAlignment valign) {
  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);
  Eigen::Vector3d win;
  gl::Project(x, y, z, T, viewport, &win);
  DrawString(s, win[0], win[1], font, red, green, blue, halign, valign);
}

//***************************************************************************
// wxWidgets implementation

#ifdef __TOOLKIT_WXWINDOWS__

#include "stdwx.h"

void RenderDrawStrings(QWidget *) {
  glBlendFunc(GL_ZERO, GL_SRC_COLOR);   // Blending mode ensures text background
  glEnable(GL_BLEND);                   //   is transparent

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

  for (int i = 0; i < strings.size(); i++) {
    StringToDraw &s = strings[i];

    // Create the memory DC and measure the text.
    wxMemoryDC memdc;
    memdc.SetFont(*s.font->font);
    wxCoord w, h, descent;
    memdc.GetTextExtent(s.s.c_str(), &w, &h, &descent);
    AlignText(w, h, descent, s.halign, s.valign, false, &s.x, &s.y);

    // Create the bitmap to render into and set text rendering mode.
    wxBitmap bitmap(w, h);                //@@@ cache bitmaps or DCs?
    memdc.SelectObject(bitmap);
    memdc.SetTextBackground(*wxWHITE);
    memdc.SetTextForeground(*wxBLACK);
    memdc.SetBackgroundMode(wxPENSTYLE_SOLID);
    memdc.DrawText(s.s.c_str(), 0, 0);
    memdc.SelectObject(wxNullBitmap);     // Now bitmap data can be used
    memdc.SetFont(wxNullFont);
    gl::SetUniform("x", s.x);
    gl::SetUniform("y", s.y);

    // Render the image.
    wxImage image = bitmap.ConvertToImage();
    gl::TextureRectangle tex(image.GetWidth(), image.GetHeight(),
                             image.GetData());
    tex.Bind();
    float iw = image.GetWidth(), ih = image.GetHeight();
    float xy[][2] = {{0, 0}, {iw, 0}, {0, ih}, {iw, ih}};
    gl::VertexBuffer<float[2]> buffer(4, xy);
    buffer.Specify1("vertex", 0, 2, GL_FLOAT);
    buffer.Draw(GL_TRIANGLE_STRIP);
  }
  glDisable(GL_BLEND);

  strings.clear();
}

#endif  // __TOOLKIT_WXWINDOWS__

//***************************************************************************
// Qt implementation

#ifdef QT_CORE_LIB

#include <QPainter>
#include <QWidget>
#include <QFont>

void RenderDrawStrings(QWidget *win) {
  double scale = win->devicePixelRatio();
  QPainter painter(win);
  for (int i = 0; i < strings.size(); i++) {
    StringToDraw &s = strings[i];

    // Convert from opengl pixel coordinates to Qt window coordinates.
    double x = s.x / scale;
    double y = win->height() - s.y / scale;

    // Measure and align the text.
    double w = s.font->fm->width(s.s.c_str());
    double h = s.font->fm->height();
    double ascent = s.font->fm->ascent();
    double descent = s.font->fm->descent();
    y = -y;     // Since Y coordinates increase going up in AlignText
    AlignText(w, h, descent, s.halign, s.valign, false, &x, &y);
    y = -y;

    // Draw the text.
    painter.setPen(QColor(s.red, s.green, s.blue, 255));
    painter.setFont(*s.font->font);
    painter.drawText(QPointF(x, y + ascent - 1), s.s.c_str());
  }
  strings.clear();
}

#endif  // QT_CORE_LIB
