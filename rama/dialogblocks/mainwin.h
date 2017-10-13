/////////////////////////////////////////////////////////////////////////////
// Name:        mainwin.h
// Purpose:
// Author:
// Modified by:
// Created:
// RCS-ID:
// Copyright:
// Licence:
/////////////////////////////////////////////////////////////////////////////

#ifndef _MAINWIN_H_
#define _MAINWIN_H_


/*!
 * Includes
 */

////@begin includes
#include "wx/frame.h"
#include "wx/splitter.h"
#include "wx/statline.h"
#include "wx/aui/auibook.h"
#include "wx/spinctrl.h"
#include "wx/listctrl.h"
////@end includes

/*!
 * Forward declarations
 */

////@begin forward declarations
class wxFlexGridSizer;
class wxAuiNotebook;
class wxBoxSizer;
class wxSpinCtrl;
class Cavity;
class wxPlot;
class wxListCtrl;
////@end forward declarations

/*!
 * Control identifiers
 */

////@begin control identifiers
#define ID_MENU_RELOAD 10016
#define ID_MENU_AUTORUN 10015
#define ID_MENU_EXPORT_DXF 10002
#define ID_MENU_EXPORT_XY 10003
#define ID_MENU_EXPORT_FIELD_MATLAB 10010
#define ID_MENU_EXPORT_PLOT_MATLAB 10036
#define ID_MENU_VIEW_ZOOM_EXTENTS 10000
#define ID_MENU_TOGGLE_BOUNDARY 10007
#define ID_MENU_TOGGLE_BOUNDARY_VERTICES 10001
#define ID_MENU_TOGGLE_BOUNDARY_DERIVATIVES 10009
#define ID_MENU_TOGGLE_GRID 10017
#define ID_MENU_TOGGLE_MARKERS 10032
#define ID_MENU_TOGGLE_ANTIALIASING 10018
#define ID_MENU_TOGGLE_EMIT_TRACE_REPORT 10026
#define ID_MENU_SOLVE_SWEEP 10011
#define ID_MENU_OPTIMIZER_LEVENBERG_MARQUARDT 10028
#define ID_MENU_OPTIMIZER_DOGLEG 10027
#define ID_MENU_OPTIMIZER_NELDER_MEAD 10029
#define ID_MENU_OPTIMIZER_SIMULATED_ANNEALING 10030
#define ID_MENU_OPTIMIZE 10012
#define ID_MENU_STOP_SOLVE 10013
#define ID_MENU_HELP_MANUAL 10022
#define ID_MENU_HELP_WEBSITE 10024
#define ID_MENU_HELP_LUA_MANUAL 10031
#define ID_MENU_HELP_INSTALL_LATEST 10004
#define ID_MENU_HELP_ABOUT 10014
#define ID_CHECKBOX_RESET_PARAMS 10008
#define ID_CHOICE_MESH 10033
#define ID_CHECKBOX_FIELD 10005
#define ID_CHECKBOX_ANIMATE 10006
#define ID_SELECTION_DISPLAY_STYLE 10019
#define ID_SPIN_MODE_NUMBER 10037
#define ID_SELECTION_COLORMAP 10020
#define ID_SLIDER_BRIGHTNESS 10021
#define ID_SELECT_PLOT 10025
#define ID_ANTENNA_SHOW 10034
#define ID_ANTENNA_SCALE_MAX 10035
#define SYMBOL_MAINWIN_STYLE wxCAPTION|wxRESIZE_BORDER|wxSYSTEM_MENU|wxMINIMIZE_BOX|wxMAXIMIZE_BOX|wxCLOSE_BOX|wxCLIP_CHILDREN
#define SYMBOL_MAINWIN_TITLE _("Rama")
#define SYMBOL_MAINWIN_IDNAME wxID_ANY
#define SYMBOL_MAINWIN_SIZE wxSize(1100, 700)
#define SYMBOL_MAINWIN_POSITION wxDefaultPosition
////@end control identifiers


/*!
 * MainWin class declaration
 */

class MainWin: public wxFrame
{
    DECLARE_CLASS( MainWin )
    DECLARE_EVENT_TABLE()

public:
    /// Constructors
    MainWin();
    MainWin( wxWindow* parent, wxWindowID id = SYMBOL_MAINWIN_IDNAME, const wxString& caption = SYMBOL_MAINWIN_TITLE, const wxPoint& pos = SYMBOL_MAINWIN_POSITION, const wxSize& size = SYMBOL_MAINWIN_SIZE, long style = SYMBOL_MAINWIN_STYLE );

    bool Create( wxWindow* parent, wxWindowID id = SYMBOL_MAINWIN_IDNAME, const wxString& caption = SYMBOL_MAINWIN_TITLE, const wxPoint& pos = SYMBOL_MAINWIN_POSITION, const wxSize& size = SYMBOL_MAINWIN_SIZE, long style = SYMBOL_MAINWIN_STYLE );

    /// Destructor
    ~MainWin();

    /// Initialises member variables
    void Init();

    /// Creates the controls and sizers
    void CreateControls();

////@begin MainWin event handler declarations

    /// wxEVT_COMMAND_MENU_SELECTED event handler for wxID_OPEN
    void OnMenuOpen( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for ID_MENU_RELOAD
    void OnMenuReload( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for ID_MENU_AUTORUN
    void OnMenuAutorun( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for ID_MENU_EXPORT_DXF
    void OnMenuExportDxf( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for ID_MENU_EXPORT_XY
    void OnMenuExportXy( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for ID_MENU_EXPORT_FIELD_MATLAB
    void OnMenuExportFieldMatlab( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for ID_MENU_EXPORT_PLOT_MATLAB
    void OnMenuExportPlotMatlabClick( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for wxID_EXIT
    void OnMenuExit( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for wxID_COPY
    void OnCopy( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for ID_MENU_VIEW_ZOOM_EXTENTS
    void OnMenuViewZoomExtents( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for wxID_ZOOM_IN
    void OnZoomIn( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for wxID_ZOOM_OUT
    void OnZoomOut( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for ID_MENU_TOGGLE_BOUNDARY
    void OnMenuToggleBoundary( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for ID_MENU_TOGGLE_BOUNDARY_VERTICES
    void OnMenuToggleBoundaryVertices( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for ID_MENU_TOGGLE_BOUNDARY_DERIVATIVES
    void OnMenuToggleBoundaryDerivatives( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for ID_MENU_TOGGLE_GRID
    void OnMenuToggleGrid( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for ID_MENU_TOGGLE_MARKERS
    void OnMenuToggleMarkers( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for ID_MENU_TOGGLE_ANTIALIASING
    void OnMenuToggleAntialiasing( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for ID_MENU_TOGGLE_EMIT_TRACE_REPORT
    void OnMenuToggleEmitTraceReport( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for ID_MENU_SOLVE_SWEEP
    void OnMenuSolveSweep( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for ID_MENU_OPTIMIZER_LEVENBERG_MARQUARDT
    void OnMenuOptimizerLevenbergMarquardt( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for ID_MENU_OPTIMIZER_DOGLEG
    void OnMenuOptimizerDogleg( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for ID_MENU_OPTIMIZE
    void OnMenuOptimize( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for ID_MENU_STOP_SOLVE
    void OnMenuStopSolve( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for ID_MENU_HELP_MANUAL
    void OnMenuHelpManual( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for ID_MENU_HELP_WEBSITE
    void OnMenuHelpWebsite( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for ID_MENU_HELP_LUA_MANUAL
    void OnMenuHelpLuaManual( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for ID_MENU_HELP_INSTALL_LATEST
    void OnMenuHelpInstallLatest( wxCommandEvent& event );

    /// wxEVT_COMMAND_MENU_SELECTED event handler for ID_MENU_HELP_ABOUT
    void OnMenuHelpAbout( wxCommandEvent& event );

    /// wxEVT_COMMAND_CHECKBOX_CLICKED event handler for ID_CHECKBOX_RESET_PARAMS
    void OnCheckboxResetParams( wxCommandEvent& event );

    /// wxEVT_COMMAND_CHOICE_SELECTED event handler for ID_CHOICE_MESH
    void OnChoiceMesh( wxCommandEvent& event );

    /// wxEVT_COMMAND_CHECKBOX_CLICKED event handler for ID_CHECKBOX_FIELD
    void OnViewFieldClick( wxCommandEvent& event );

    /// wxEVT_COMMAND_CHECKBOX_CLICKED event handler for ID_CHECKBOX_ANIMATE
    void OnAnimateClick( wxCommandEvent& event );

    /// wxEVT_COMMAND_CHOICE_SELECTED event handler for ID_SELECTION_DISPLAY_STYLE
    void OnDisplayStyleSelected( wxCommandEvent& event );

    /// wxEVT_COMMAND_SPINCTRL_UPDATED event handler for ID_SPIN_MODE_NUMBER
    void OnSpinModeNumberUpdated( wxSpinEvent& event );

    /// wxEVT_COMMAND_CHOICE_SELECTED event handler for ID_SELECTION_COLORMAP
    void OnDisplayColorSelected( wxCommandEvent& event );

    /// wxEVT_SCROLL_* event handler for ID_SLIDER_BRIGHTNESS
    void OnSliderBrightnessScroll( wxScrollEvent& event );

    /// wxEVT_COMMAND_CHOICE_SELECTED event handler for ID_SELECT_PLOT
    void OnSelectPlot( wxCommandEvent& event );

    /// wxEVT_COMMAND_CHECKBOX_CLICKED event handler for ID_ANTENNA_SHOW
    void OnAntennaShow( wxCommandEvent& event );

    /// wxEVT_COMMAND_CHECKBOX_CLICKED event handler for ID_ANTENNA_SCALE_MAX
    void OnAntennaScaleMax( wxCommandEvent& event );

////@end MainWin event handler declarations

////@begin MainWin member function declarations

    /// Retrieves bitmap resources
    wxBitmap GetBitmapResource( const wxString& name );

    /// Retrieves icon resources
    wxIcon GetIconResource( const wxString& name );
////@end MainWin member function declarations

    /// Should we show tooltips?
    static bool ShowToolTips();

////@begin MainWin member variables
    wxStaticText* script_filename_label_;
    wxScrolledWindow* parameter_scrolledwindow_;
    wxFlexGridSizer* parameter_flexgrid_;
    wxAuiNotebook* aui_notebook_;
    wxBoxSizer* display_controls_sizer_;
    wxCheckBox* view_field_checkbox_;
    wxCheckBox* animate_checkbox_;
    wxChoice* display_style_Ez_;
    wxChoice* display_style_Exy_;
    wxChoice* display_style_TM_;
    wxChoice* display_style_TE_;
    wxStaticText* mode_number_label_;
    wxSpinCtrl* mode_number_;
    Cavity* viewer_;
    wxPlot* plot_;
    wxChoice* plot_showing_;
    wxPlot* antenna_pattern_;
    wxListCtrl* script_messages_;
////@end MainWin member variables

    wxFileName script_filename_;            // Currently opened file
    wxTimer current_model_label_timer_;     // Delivers OnCurrentModelLabelTimeout()
    wxTimer reload_timer_;                  // Delivers OnReloadTimeout()
    wxFileSystemWatcher *model_watcher_;    // Looks for model directory changes
    bool autorun_;                          // True to rerun script when file changes

    void DoInitializationThatRequiresAnEventLoop();
    void DoShutdownThatRequiresAnEventLoop();
    void OnCurrentModelLabelTimeout(wxTimerEvent& event);
    void OnReloadTimeout(wxTimerEvent& event);
    void OnModelFileChanged(wxFileSystemWatcherEvent& event);

    void LoadFile(const wxString &full_path);
    bool ReloadScript(bool rerun_even_if_same);  // Return true if file loadable
};

#endif
    // _MAINWIN_H_
