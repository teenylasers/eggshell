
#include "stdwx.h"
#include "common.h"

//***************************************************************************
// Common fonts.

const wxFont *port_number_font = 0, *s_parameter_font = 0,
             *mesh_statistics_font = 0;

void CreateStandardFonts(double content_scale_factor) {
  if (!port_number_font) {
    double scale = content_scale_factor * FONT_SCALE;
    port_number_font = new wxFont(10 * scale, wxFONTFAMILY_TELETYPE,
                                  wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD);
    s_parameter_font = new wxFont(10 * scale, wxFONTFAMILY_SWISS,
                                  wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL);
    mesh_statistics_font = new wxFont(10 * scale, wxFONTFAMILY_SWISS,
                                      wxFONTSTYLE_ITALIC, wxFONTWEIGHT_NORMAL);
  }
}
