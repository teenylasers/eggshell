
#include "stdwx.h"
#include <wx/dcbuffer.h>
#include <wx/graphics.h>
#include "plot_wx.h"
#include "error.h"

//***************************************************************************
// Plot::Window interface to a wxWindow.

namespace Plot {

#ifdef __APPLE__
  const int kLabelFontSize = 16;
  const int kLabelSmallFontSize = 12;
#else
  const int kLabelFontSize = 10;
  const int kLabelSmallFontSize = 8;
#endif

class PlotWindow : public Plot::Window {
 public:
  wxWindow *win;
  wxDC *dc;
  wxGraphicsContext *gc;
  Style style;
  bool antialias_lines;
  int color;
  int width;
  double last_x2, last_y2;
  bool last_x2y2_valid;
  mutable bool refreshed;
  int window_width, window_height;
  const wxFont *current_font, *label_font, *label_small_font;
  wxGraphicsPath path;

  PlotWindow(wxWindow *_win, wxDC *_dc, wxGraphicsContext *_gc) {
    win = _win;
    label_font = new wxFont(kLabelFontSize, wxFONTFAMILY_SWISS,
                            wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL);
    label_small_font = new wxFont(kLabelSmallFontSize, wxFONTFAMILY_SWISS,
                                  wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL);
    Reset(_dc, _gc);
  }

  // This is called before drawing to reset state.
  void Reset(wxDC *_dc, wxGraphicsContext *_gc) {
    dc = _dc;
    gc = _gc;
    style = BORDER;
    antialias_lines = false;
    color = 0;
    width = 1;
    last_x2 = last_y2 = 0;
    last_x2y2_valid = false;
    refreshed = false;
    win->GetClientSize(&window_width, &window_height);
    UseLabelFont();
    if (dc) {
      dc->SetTextBackground(wxTransparentColour);
    }
  }

  // This is called immediately after drawing.
  void DoneDrawing() {
    dc->SetPen(wxNullPen);
    dc->SetBrush(wxNullBrush);
    dc->SetFont(wxNullFont);
  }

  ~PlotWindow() {
    delete label_font;
    delete label_small_font;
  }

  double Width() const {
    return window_width;
  }

  double Height() const {
    return window_height;
  }

  void Refresh() const {
    win->Refresh();
    refreshed = true;
  }

  void Color(uint32_t rrggbb) {
    rrggbb = ((rrggbb & 0xff) << 16) |
              (rrggbb & 0xff00) |
             ((rrggbb & 0xff0000) >> 16);
    color = rrggbb;
    dc->SetTextForeground(wxColour(rrggbb));
  }

  void LineWidth(double w) {
    width = w;
  }

  void StartLine(Style s) {
    style = s;
    antialias_lines = false;            // Default (BORDER style)
    #ifdef __APPLE__
      // OS X uses subpixel antialiased stroked paths even for wxDC, so to
      // get enough control to stroke single-pixel-width lines we need to
      // use wxGraphicsContext everywhere.
      antialias_lines = true;           // Default on OS X
    #endif
    wxPenStyle wxpenstyle = wxPENSTYLE_SOLID;   // Default (BORDER style)
    if (s == TRACE) {
      wxpenstyle = wxPENSTYLE_SOLID;
      antialias_lines = true;
    } else if (s == GRID) {
      wxpenstyle = wxPENSTYLE_DOT;
    }
    wxPen *pen = wxThePenList->FindOrCreatePen(wxColour(color), width,
                                               wxpenstyle);
    if (antialias_lines) {
      gc->SetPen(*pen);
      path = gc->CreatePath();
    } else {
      dc->SetPen(*pen);
    }
    last_x2y2_valid = false;
  }

  void EndLine() {
    if (antialias_lines) {
      gc->StrokePath(path);
    }
  }

  void Line(double x1, double y1, double x2, double y2) {
    if (style == BORDER || style == GRID) {
      // This is necessary for OS X to achieve single-pixel-width lines.
      x1 = round(x1);
      y1 = round(y1);
      x2 = round(x2);
      y2 = round(y2);
    }

    y1 = window_height - 1 - y1;
    y2 = window_height - 1 - y2;
    if (antialias_lines) {
      if (last_x2y2_valid && x1 == last_x2 && y1 == last_y2) {
        // We're already at x1, y1.
      } else {
        path.MoveToPoint(x1, y1);
      }
      path.AddLineToPoint(x2, y2);
    } else {
      dc->DrawLine(x1, y1, x2, y2);
    }
    last_x2 = x2;
    last_y2 = y2;
    last_x2y2_valid = true;
  }

  void Rectangle(double x, double y, double w, double h) {
    y = window_height - y - h;
    dc->SetBrush(*wxTheBrushList->FindOrCreateBrush(wxColour(color)));
    dc->SetPen(*wxTRANSPARENT_PEN);
    dc->DrawRectangle(x, y, w, h);
  }

  void DrawText(const char *s, double x, double y,
                TextAlignment halign, TextAlignment valign, bool rotate_90) {
    int width, height, descent, leading;
    dc->GetTextExtent(s, &width, &height, &descent, &leading, current_font);

    ::AlignText(width, height, descent, halign, valign, rotate_90, &x, &y);

    y = window_height - 1 - y;
    if (rotate_90) {
      dc->DrawRotatedText(s, x, y + 1, 90);
    } else {
      dc->DrawText(s, x, y);
    }
  }

  void SelectionRectangle(double x1, double y1, double x2, double y2,
                          bool state) {
    y1 = window_height - 1 - y1;
    y2 = window_height - 1 - y2;
    #ifdef __APPLE__
      if (state) {
        dc->SetPen(*wxGREY_PEN);
        dc->SetBrush(wxColour(255, 192, 192, 64));
        dc->DrawRectangle(std::min(x1, x2), std::min(y1, y2),
                          fabs(x2 - x1), fabs(y2 - y1));
      }
    #else
      // This approach (with SetLogicalFunction) does not work on OS X.
      dc->SetPen(*wxThePenList->FindOrCreatePen(wxColour(0UL), 1,
                                                wxPENSTYLE_SHORT_DASH));
      dc->SetLogicalFunction(wxINVERT);
      dc->DrawLine(x1, y1, x2, y1);
      dc->DrawLine(x1, y1, x1, y2);
      dc->DrawLine(x2, y2, x2, y1);
      dc->DrawLine(x2, y2, x1, y2);
      dc->SetLogicalFunction(wxCOPY);
    #endif
  }

  Handle CreateImage(int width, int height, uint8_t *rgb_data) {
    CHECK(width > 0 && height > 0)
    wxImage image(width, height, rgb_data, true);       // static_data = true
    Handle h;
    h.width = width;
    h.height = height;
    h.pointer = new wxBitmap(image);
    return h;
  }

  void DestroyImage(Handle h) {
    if (h.pointer) {
      delete reinterpret_cast<wxBitmap*>(h.pointer);
    }
  }

  void DrawImage(Handle h, double x, double y, double width, double height) {
    y = window_height - 1 - y;
    wxBitmap *bitmap = reinterpret_cast<wxBitmap*>(h.pointer);
    gc->SetInterpolationQuality(wxINTERPOLATION_NONE);
    gc->DrawBitmap(*bitmap, x, y, width, height);
  }

  void UseLabelFont() {
    current_font = label_font;
    if (dc) {
      dc->SetFont(*label_font);
    }
  }

  void UseLabelSmallFont() {
    current_font = label_small_font;
    dc->SetFont(*label_small_font);
  }

  double TextWidth(const char *s) {
    int width, height, descent, leading;
    dc->GetTextExtent(s, &width, &height, &descent, &leading, current_font);
    return width;
  }

  double TextHeight(double *descent) {
    wxFontMetrics m = dc->GetFontMetrics();
    if (descent) {
      *descent = m.descent;
    }
    return m.height;
  }
};

}  // Plot namespace

//***************************************************************************
// wxPlot.

BEGIN_EVENT_TABLE(wxPlot,wxWindow)
  EVT_PAINT(wxPlot::OnPaint)
  EVT_SIZE(wxPlot::OnSize)
  EVT_MOUSE_EVENTS(wxPlot::OnMouseEvent)
  EVT_MOUSE_CAPTURE_LOST(wxPlot::OnCaptureLost)
END_EVENT_TABLE()

wxPlot::wxPlot(wxWindow* parent, wxWindowID id, const wxPoint &pos,
               const wxSize &size, long style)
    : wxWindow(parent, id, pos, size, style) {
  plot_window_ = new Plot::PlotWindow(this, 0, 0);
  plot_ = new Plot::Plot2D(plot_window_);
  snapshot_ = 0;
  SetBackgroundStyle(wxBG_STYLE_PAINT);
}

wxPlot::~wxPlot() {
  delete snapshot_;
  delete plot_;
  delete plot_window_;
}

void wxPlot::OnPaint(wxPaintEvent &event) {
  // Use double buffering so there will be no flicker. On platforms that have
  // native double buffering (e.g. OS X) the wxAutoBufferedPaintDC is the same
  // as wxPaintDC, otherwise it is wxBufferedPaintDC . Note that for retina
  // displays wxPaintDC is also needed to get full resolution.
  wxAutoBufferedPaintDC dc(this);
  wxGraphicsContext *gc = wxGraphicsContext::Create(dc);
  CHECK(gc);
  plot_window_->Reset(&dc, gc);
  plot_->Draw();
  plot_window_->DoneDrawing();
  delete gc;
}

void wxPlot::OnSize(wxSizeEvent &evt) {
  Refresh();
  evt.Skip();   // Needed to make sure layout runs for sub-controls
}

void wxPlot::OnMouseEvent(wxMouseEvent &event) {
  // Ignore enter/leave events.
  if (event.GetEventType() == wxEVT_ENTER_WINDOW ||
      event.GetEventType() == wxEVT_LEAVE_WINDOW) {
    return;
  }

  int button_state = 0;
  if (event.LeftIsDown()) button_state |= 1;
  if (event.MiddleIsDown()) button_state |= 2;
  if (event.RightIsDown()) button_state |= 4;

  // Get mouse position in plot coordinates (Y inverted from window
  // coordinates).
  int width, height;
  GetClientSize(&width, &height);
  int x = event.GetX();
  int y = height - 1 - event.GetY();

  if (button_state && !HasCapture()) {
    CaptureMouse();
  }
  if (button_state == 0 && HasCapture()) {
    ReleaseMouse();
  }

  static int last_button_state = 0;
  if (last_button_state || button_state) {
    wxClientDC dc(this);
    wxGraphicsContext *gc = wxGraphicsContext::Create(dc);
    CHECK(gc);
    #ifdef __APPLE__
      if (last_button_state == 0 && button_state != 0) {
        SaveSnapshot(&dc);
      } else if (button_state != 0) {
        RestoreSnapshot(&dc);
      }
    #endif
    plot_window_->Reset(&dc, gc);
    plot_->EventMouse(x, y, button_state);
    #ifdef __APPLE__
      if (plot_window_->refreshed) {
        RestoreSnapshot(&dc);
        DeleteSnapshot();
      }
    #endif
    plot_window_->DoneDrawing();
    delete gc;
  }
  last_button_state = button_state;
}

void wxPlot::OnCaptureLost(wxMouseCaptureLostEvent &event) {
  // Required to prevent capture handler from complaining.
}

void wxPlot::SaveSnapshot(wxDC *dc) {
  // This function (and friends) implement more or less the same functionality
  // as wxOverlay, but have the added advantage that they work with retina
  // displays on OS X. On a retina display the scale is 2, i.e. 2 pixels per
  // "point", so we must adjust the size of the bitmap and Blit() arguments.
  DeleteSnapshot();
  int width, height;
  GetClientSize(&width, &height);
  double scale = GetContentScaleFactor();
  snapshot_ = new wxBitmap(width * scale, height * scale);
  wxMemoryDC memdc;
  memdc.SelectObject(*snapshot_);
  memdc.StretchBlit(0, 0, width*scale, height*scale, dc, 0, 0,
                    width, height);
  memdc.SelectObject(wxNullBitmap);
}

void wxPlot::RestoreSnapshot(wxDC *dc) {
  if (snapshot_) {
    int width, height;
    GetClientSize(&width, &height);
    wxMemoryDC memdc;
    memdc.SelectObjectAsSource(*snapshot_);
    double scale = GetContentScaleFactor();
    if (scale == 1) {
      dc->Blit(0, 0, width, height, &memdc, 0, 0);
    } else {
      // This is likely a retina display with scale 2, i.e. 2 pixels per
      // "point". The width,height are in points. The xdest,ydest arguments to
      // StretchBlit are in points, the other arguments are in pixels.
      dc->StretchBlit(0, 0, width, height, &memdc, 0, 0,
                      width*scale, height*scale);
    }
    memdc.SelectObject(wxNullBitmap);
  }
}

void wxPlot::DeleteSnapshot() {
  delete snapshot_;
  snapshot_ = 0;
}
