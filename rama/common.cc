
#define GL_FONT_IMPLEMENTATION

#include "stdwx.h"
#include "common.h"

//***************************************************************************
// Common fonts.

Font port_number_font;
Font s_parameter_font;
Font mesh_statistics_font;
Font debug_string_font;

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
