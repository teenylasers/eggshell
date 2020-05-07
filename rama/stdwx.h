// Rama Simulator, Copyright (C) 2014-2020 Russell Smith.
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

#ifndef __STD_WX_H_PRECOMPILED_HEADER__
#define __STD_WX_H_PRECOMPILED_HEADER__

// Eigen will complain if its included after wx headers, because 'Success' gets
// defined as a macro somewhere and eigen has an explicit check for this. So
// include it here.
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "wx/wx.h"
#include "wx/splitter.h"
#include "wx/listctrl.h"
#include "wx/artprov.h"
#include "wx/aui/auibook.h"
#include "wx/fswatcher.h"
#include <wx/clipbrd.h>
#include "wx/statline.h"

// Including glcanvas.h can bring in GL headers, which on windows and linux
// will include only basic GL functionality unless this is defined:
#define GL_GLEXT_PROTOTYPES

// On linux the glcanvas include brings in X11 headers, which pollute the macro
// namespace  with all kinds of commonly used words (e.g. Window, Complex,
// True). Include undef_x11.h to undefine all macros found in /usr/include/X11
// that don't start with an underscore.
#include "wx/glcanvas.h"
#ifdef __WXGTK__
#include "undef_x11.h"
#endif

#endif
