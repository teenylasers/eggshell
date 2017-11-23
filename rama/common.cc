
#define GL_FONT_IMPLEMENTATION

#include "common.h"

//***************************************************************************
// Common fonts.

Font port_number_font;
Font s_parameter_font;
Font mesh_statistics_font;
Font debug_string_font;

#ifdef __TOOLKIT_WXWINDOWS__

#include "stdwx.h"

void CreateStandardFonts(double content_scale_factor) {
  if (!port_number_font.font) {
    double scale = content_scale_factor * FONT_SCALE;
    port_number_font.font = new wxFont(10 * scale, wxFONTFAMILY_TELETYPE,
                                       wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD);
    s_parameter_font.font = new wxFont(10 * scale, wxFONTFAMILY_SWISS,
                                       wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL);
    mesh_statistics_font.font = new wxFont(10 * scale, wxFONTFAMILY_SWISS,
                                       wxFONTSTYLE_ITALIC, wxFONTWEIGHT_NORMAL);
    debug_string_font.font = new wxFont(10 * scale, wxFONTFAMILY_SWISS,
                                       wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL);
  }
}

#endif  // __TOOLKIT_WXWINDOWS__

#ifdef QT_CORE_LIB

#include <QFont>
#include <QFontMetrics>

void CreateStandardFonts(double content_scale_factor) {
  if (!port_number_font.font) {
    port_number_font.font = new QFont("Courier", 10, QFont::Bold);
    port_number_font.font->setStyleHint(QFont::TypeWriter);
    port_number_font.fm = new QFontMetrics(*port_number_font.font);

    s_parameter_font.font = new QFont("Helvetica", 10, QFont::Normal);
    s_parameter_font.font->setStyleHint(QFont::QFont::SansSerif);
    s_parameter_font.fm = new QFontMetrics(*s_parameter_font.font);

    mesh_statistics_font.font = new QFont("Helvetica", 10, QFont::Normal, true);
    mesh_statistics_font.font->setStyleHint(QFont::QFont::SansSerif);
    mesh_statistics_font.fm = new QFontMetrics(*mesh_statistics_font.font);

    debug_string_font.font = new QFont("Helvetica", 10, QFont::Normal);
    debug_string_font.font->setStyleHint(QFont::QFont::SansSerif);
    debug_string_font.fm = new QFontMetrics(*debug_string_font.font);
  }
}

#endif
