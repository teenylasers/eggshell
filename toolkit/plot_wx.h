
// wxWidgets interface to the plotting system.

#ifndef __TOOLKIT_PLOT_WX_H__
#define __TOOLKIT_PLOT_WX_H__

#include "plot.h"

class wxBitmap;
namespace Plot { class PlotWindow; }

class wxPlot : public wxWindow {
 public:
  wxPlot(wxWindow* parent, wxWindowID id, const wxPoint &pos,
         const wxSize &size, long style);
  ~wxPlot();

  // Interaction with the plot.
  Plot::Plot2D &plot() { return *plot_; }

  // wx event handling.
  void OnPaint(wxPaintEvent &event);
  void OnSize(wxSizeEvent &evt);
  void OnMouseEvent(wxMouseEvent &event);
  void OnCaptureLost(wxMouseCaptureLostEvent &event);

 private:
  Plot::PlotWindow *plot_window_;
  Plot::Plot2D *plot_;
  wxBitmap *snapshot_;          // For rubber-banding on OS X

  void SaveSnapshot(wxDC *dc);
  void RestoreSnapshot(wxDC *dc);
  void DeleteSnapshot();

  DECLARE_EVENT_TABLE()
};

#endif
