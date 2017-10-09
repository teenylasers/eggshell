// OpenGL font support using wxWidgets.

#ifndef __TOOLKIT_WXGL_FONT_H__
#define __TOOLKIT_WXGL_FONT_H__

#include "text_alignment.h"

class wxFont;
class wxWindow;

// Draw a string to pixel coordinates (x,y) with (0,0) being the bottom left of
// the viewport. The caller should set drawing modes (e.g. color). Note that
// this changes the glBlendFunc() and leaves GL_BLEND disabled on exit.
void DrawString(const char *s, double x, double y, const wxFont &font,
    TextAlignment halign = TEXT_ALIGN_LEFT,
    TextAlignment valign = TEXT_ALIGN_BASELINE);

// Like DrawString, but (x,y,z) are in model coordinates with respect to the
// given transform matrix.
void DrawStringM(const char *s, double x, double y, double z,
                 const Eigen::Matrix4d &T, const wxFont &font,
                 TextAlignment halign = TEXT_ALIGN_LEFT,
                 TextAlignment valign = TEXT_ALIGN_BASELINE);

#endif
