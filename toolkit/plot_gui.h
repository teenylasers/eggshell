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

// GUI interface to the plotting system.

#ifndef __TOOLKIT_PLOT_GUI_H__
#define __TOOLKIT_PLOT_GUI_H__

#include "plot.h"

//***************************************************************************
// wxWidgets.

#ifdef __TOOLKIT_WXWINDOWS__

#include "stdwx.h"

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

#endif  // __TOOLKIT_WXWINDOWS__

//***************************************************************************
// Qt

#ifdef QT_CORE_LIB

#include <QWidget>

namespace Plot { class PlotWindow; }

class qtPlot : public QWidget {
 public:
  qtPlot(QWidget* parent);
  ~qtPlot();

  // Interaction with the plot.
  Plot::Plot2D &plot() { return *plot_; }

  // Event handling.
  void paintEvent(QPaintEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void mouseDoubleClickEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void HandleMouse(QMouseEvent *event);

 private:
  Plot::PlotWindow *plot_window_;
  Plot::Plot2D *plot_;
};

#endif  // QT_CORE_LIB

#endif
