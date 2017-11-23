/////////////////////////////////////////////////////////////////////////////
// Name:        mainwin.cc
// Purpose:
// Author:
// Modified by:
// Created:
// RCS-ID:
// Copyright:
// Licence:
/////////////////////////////////////////////////////////////////////////////

// Precompiled header:
#include "../stdwx.h"
#include <wx/stdpaths.h>
#include <wx/uri.h>

////@begin includes
#include "wx/imaglist.h"
////@end includes

#include "mainwin.h"
#include "aboutwin.h"
#include "sweep.h"
#include "../cavity.h"
#include "../../toolkit/plot_gui.h"
#include "../version.h"
#include "../../toolkit/optimizer.h"

////@begin XPM images
////@end XPM images


/*
 * MainWin type definition
 */

IMPLEMENT_CLASS( MainWin, wxFrame )


/*
 * MainWin event table definition
 */

enum {
  CURRENT_MODE_LABEL_TIMER_ID = 10,
  RELOAD_TIMER_ID = 11,
};

BEGIN_EVENT_TABLE( MainWin, wxFrame )
////@begin MainWin event table entries
    EVT_MENU( wxID_OPEN, MainWin::OnMenuOpen )
    EVT_MENU( ID_MENU_RELOAD, MainWin::OnMenuReload )
    EVT_MENU( ID_MENU_AUTORUN, MainWin::OnMenuAutorun )
    EVT_MENU( ID_MENU_EXPORT_DXF, MainWin::OnMenuExportDxf )
    EVT_MENU( ID_MENU_EXPORT_XY, MainWin::OnMenuExportXy )
    EVT_MENU( ID_MENU_EXPORT_FIELD_MATLAB, MainWin::OnMenuExportFieldMatlab )
    EVT_MENU( ID_MENU_EXPORT_PLOT_MATLAB, MainWin::OnMenuExportPlotMatlabClick )
    EVT_MENU( wxID_EXIT, MainWin::OnMenuExit )
    EVT_MENU( wxID_COPY, MainWin::OnCopy )
    EVT_MENU( ID_MENU_VIEW_ZOOM_EXTENTS, MainWin::OnMenuViewZoomExtents )
    EVT_MENU( wxID_ZOOM_IN, MainWin::OnZoomIn )
    EVT_MENU( wxID_ZOOM_OUT, MainWin::OnZoomOut )
    EVT_MENU( ID_MENU_TOGGLE_BOUNDARY, MainWin::OnMenuToggleBoundary )
    EVT_MENU( ID_MENU_TOGGLE_BOUNDARY_VERTICES, MainWin::OnMenuToggleBoundaryVertices )
    EVT_MENU( ID_MENU_TOGGLE_BOUNDARY_DERIVATIVES, MainWin::OnMenuToggleBoundaryDerivatives )
    EVT_MENU( ID_MENU_TOGGLE_GRID, MainWin::OnMenuToggleGrid )
    EVT_MENU( ID_MENU_TOGGLE_MARKERS, MainWin::OnMenuToggleMarkers )
    EVT_MENU( ID_MENU_TOGGLE_ANTIALIASING, MainWin::OnMenuToggleAntialiasing )
    EVT_MENU( ID_MENU_TOGGLE_EMIT_TRACE_REPORT, MainWin::OnMenuToggleEmitTraceReport )
    EVT_MENU( ID_MENU_SOLVE_SWEEP, MainWin::OnMenuSolveSweep )
    EVT_MENU( ID_MENU_OPTIMIZER_LEVENBERG_MARQUARDT, MainWin::OnMenuOptimizerLevenbergMarquardt )
    EVT_MENU( ID_MENU_OPTIMIZER_DOGLEG, MainWin::OnMenuOptimizerDogleg )
    EVT_MENU( ID_MENU_OPTIMIZE, MainWin::OnMenuOptimize )
    EVT_MENU( ID_MENU_STOP_SOLVE, MainWin::OnMenuStopSolve )
    EVT_MENU( ID_MENU_HELP_MANUAL, MainWin::OnMenuHelpManual )
    EVT_MENU( ID_MENU_HELP_WEBSITE, MainWin::OnMenuHelpWebsite )
    EVT_MENU( ID_MENU_HELP_LUA_MANUAL, MainWin::OnMenuHelpLuaManual )
    EVT_MENU( ID_MENU_HELP_INSTALL_LATEST, MainWin::OnMenuHelpInstallLatest )
    EVT_MENU( ID_MENU_HELP_ABOUT, MainWin::OnMenuHelpAbout )
    EVT_CHECKBOX( ID_CHECKBOX_RESET_PARAMS, MainWin::OnCheckboxResetParams )
    EVT_CHOICE( ID_CHOICE_MESH, MainWin::OnChoiceMesh )
    EVT_CHECKBOX( ID_CHECKBOX_FIELD, MainWin::OnViewFieldClick )
    EVT_CHECKBOX( ID_CHECKBOX_ANIMATE, MainWin::OnAnimateClick )
    EVT_CHOICE( ID_SELECTION_DISPLAY_STYLE, MainWin::OnDisplayStyleSelected )
    EVT_SPINCTRL( ID_SPIN_MODE_NUMBER, MainWin::OnSpinModeNumberUpdated )
    EVT_CHOICE( ID_SELECTION_COLORMAP, MainWin::OnDisplayColorSelected )
    EVT_COMMAND_SCROLL( ID_SLIDER_BRIGHTNESS, MainWin::OnSliderBrightnessScroll )
    EVT_CHOICE( ID_SELECT_PLOT, MainWin::OnSelectPlot )
    EVT_CHECKBOX( ID_ANTENNA_SHOW, MainWin::OnAntennaShow )
    EVT_CHECKBOX( ID_ANTENNA_SCALE_MAX, MainWin::OnAntennaScaleMax )
////@end MainWin event table entries
    EVT_TIMER(CURRENT_MODE_LABEL_TIMER_ID, MainWin::OnCurrentModelLabelTimeout)
    EVT_TIMER(RELOAD_TIMER_ID, MainWin::OnReloadTimeout)
    EVT_FSWATCHER(wxID_ANY, MainWin::OnModelFileChanged)
END_EVENT_TABLE()


/*
 * MainWin constructors
 */

MainWin::MainWin()
{
    Init();
}

MainWin::MainWin( wxWindow* parent, wxWindowID id, const wxString& caption, const wxPoint& pos, const wxSize& size, long style )
    : current_model_label_timer_(this, CURRENT_MODE_LABEL_TIMER_ID),
      reload_timer_(this, RELOAD_TIMER_ID),
      autorun_(true)
{
    Init();
    Create( parent, id, caption, pos, size, style );
    model_watcher_ = 0;
}


/*
 * MainWin creator
 */

bool MainWin::Create( wxWindow* parent, wxWindowID id, const wxString& caption, const wxPoint& pos, const wxSize& size, long style )
{
////@begin MainWin creation
    wxFrame::Create( parent, id, caption, pos, size, style );

    CreateControls();
    Centre();
////@end MainWin creation
    return true;
}


/*
 * MainWin destructor
 */

MainWin::~MainWin()
{
////@begin MainWin destruction
////@end MainWin destruction
    DoShutdownThatRequiresAnEventLoop();
}


/*
 * Member initialisation
 */

void MainWin::Init()
{
////@begin MainWin member initialisation
    script_filename_label_ = NULL;
    parameter_scrolledwindow_ = NULL;
    parameter_flexgrid_ = NULL;
    aui_notebook_ = NULL;
    display_controls_sizer_ = NULL;
    view_field_checkbox_ = NULL;
    animate_checkbox_ = NULL;
    display_style_Ez_ = NULL;
    display_style_Exy_ = NULL;
    display_style_TM_ = NULL;
    display_style_TE_ = NULL;
    mode_number_label_ = NULL;
    mode_number_ = NULL;
    viewer_ = NULL;
    plot_ = NULL;
    plot_showing_ = NULL;
    antenna_pattern_ = NULL;
    script_messages_ = NULL;
////@end MainWin member initialisation
}


/*
 * Control creation for MainWin
 */

void MainWin::CreateControls()
{
////@begin MainWin content construction
    MainWin* itemFrame1 = this;

    wxMenuBar* menuBar = new wxMenuBar;
    wxMenu* itemMenu3 = new wxMenu;
    itemMenu3->Append(wxID_OPEN, _("&Open model...\tCtrl+O"), wxEmptyString, wxITEM_NORMAL);
    itemMenu3->Append(ID_MENU_RELOAD, _("&Reload model\tCtrl+R"), wxEmptyString, wxITEM_NORMAL);
    itemMenu3->Append(ID_MENU_AUTORUN, _("&Auto-run model on change"), wxEmptyString, wxITEM_CHECK);
    itemMenu3->Check(ID_MENU_AUTORUN, true);
    itemMenu3->AppendSeparator();
    itemMenu3->Append(ID_MENU_EXPORT_DXF, _("Export boundary as &DXF...\tCtrl+S"), wxEmptyString, wxITEM_NORMAL);
    itemMenu3->Append(ID_MENU_EXPORT_XY, _("Export boundary as x,&y list..."), wxEmptyString, wxITEM_NORMAL);
    itemMenu3->Append(ID_MENU_EXPORT_FIELD_MATLAB, _("Export field as matlab data..."), wxEmptyString, wxITEM_NORMAL);
    itemMenu3->Append(ID_MENU_EXPORT_PLOT_MATLAB, _("Export plot as matlab data..."), wxEmptyString, wxITEM_NORMAL);
    itemMenu3->AppendSeparator();
    itemMenu3->Append(wxID_EXIT, _("E&xit\tAlt+F4"), wxEmptyString, wxITEM_NORMAL);
    menuBar->Append(itemMenu3, _("&File"));
    wxMenu* itemMenu14 = new wxMenu;
    itemMenu14->Append(wxID_COPY, _("&Copy parameters to clipboard\tCtrl+C"), wxEmptyString, wxITEM_NORMAL);
    menuBar->Append(itemMenu14, _("&Edit"));
    wxMenu* itemMenu16 = new wxMenu;
    itemMenu16->Append(ID_MENU_VIEW_ZOOM_EXTENTS, _("Zoom to &extents\tCtrl+Z"), wxEmptyString, wxITEM_NORMAL);
    itemMenu16->Append(wxID_ZOOM_IN, _("Zoom in\tPgDn"), wxEmptyString, wxITEM_NORMAL);
    itemMenu16->Append(wxID_ZOOM_OUT, _("Zoom out\tPgUp"), wxEmptyString, wxITEM_NORMAL);
    itemMenu16->AppendSeparator();
    wxMenu* itemMenu21 = new wxMenu;
    itemMenu21->Append(ID_MENU_TOGGLE_BOUNDARY, _("&Lines and ports"), wxEmptyString, wxITEM_CHECK);
    itemMenu21->Check(ID_MENU_TOGGLE_BOUNDARY, true);
    itemMenu21->Append(ID_MENU_TOGGLE_BOUNDARY_VERTICES, _("&Vertices"), wxEmptyString, wxITEM_CHECK);
    itemMenu21->Append(ID_MENU_TOGGLE_BOUNDARY_DERIVATIVES, _("Vertex derivatives w.r.t first checked parameter"), wxEmptyString, wxITEM_CHECK);
    itemMenu16->Append(wxID_ANY, _("Show boundary"), itemMenu21);
    itemMenu16->Append(ID_MENU_TOGGLE_GRID, _("Show &grid"), wxEmptyString, wxITEM_CHECK);
    itemMenu16->Append(ID_MENU_TOGGLE_MARKERS, _("Show &markers"), wxEmptyString, wxITEM_CHECK);
    itemMenu16->Check(ID_MENU_TOGGLE_MARKERS, true);
    itemMenu16->Append(ID_MENU_TOGGLE_ANTIALIASING, _("Antialiasing"), wxEmptyString, wxITEM_CHECK);
    itemMenu16->Check(ID_MENU_TOGGLE_ANTIALIASING, true);
    itemMenu16->AppendSeparator();
    wxMenu* itemMenu29 = new wxMenu;
    itemMenu29->Append(ID_MENU_TOGGLE_EMIT_TRACE_REPORT, _("Emit trace report to stdout"), wxEmptyString, wxITEM_CHECK);
    itemMenu16->Append(wxID_ANY, _("Debugging"), itemMenu29);
    menuBar->Append(itemMenu16, _("&View"));
    wxMenu* itemMenu31 = new wxMenu;
    itemMenu31->Append(ID_MENU_SOLVE_SWEEP, _("&Sweep"), wxEmptyString, wxITEM_NORMAL);
    wxMenu* itemMenu33 = new wxMenu;
    itemMenu33->Append(ID_MENU_OPTIMIZER_LEVENBERG_MARQUARDT, _("Levenberg Marquardt"), wxEmptyString, wxITEM_RADIO);
    itemMenu33->Check(ID_MENU_OPTIMIZER_LEVENBERG_MARQUARDT, true);
    itemMenu33->Append(ID_MENU_OPTIMIZER_DOGLEG, _("Subspace dogleg"), wxEmptyString, wxITEM_RADIO);
    itemMenu33->Append(ID_MENU_OPTIMIZER_NELDER_MEAD, _("Nelder Mead"), wxEmptyString, wxITEM_RADIO);
    itemMenu33->Enable(ID_MENU_OPTIMIZER_NELDER_MEAD, false);
    itemMenu33->Append(ID_MENU_OPTIMIZER_SIMULATED_ANNEALING, _("Simulated annealing"), wxEmptyString, wxITEM_RADIO);
    itemMenu33->Enable(ID_MENU_OPTIMIZER_SIMULATED_ANNEALING, false);
    itemMenu31->Append(wxID_ANY, _("Select optimizer"), itemMenu33);
    itemMenu31->Append(ID_MENU_OPTIMIZE, _("Start o&ptimi&zation\tCtrl+P"), wxEmptyString, wxITEM_NORMAL);
    itemMenu31->AppendSeparator();
    itemMenu31->Append(ID_MENU_STOP_SOLVE, _("S&top sweep or optimization"), wxEmptyString, wxITEM_NORMAL);
    menuBar->Append(itemMenu31, _("&Solve"));
    wxMenu* itemMenu41 = new wxMenu;
    itemMenu41->Append(ID_MENU_HELP_MANUAL, _("Rama manual\tF1"), wxEmptyString, wxITEM_NORMAL);
    itemMenu41->Append(ID_MENU_HELP_WEBSITE, _("Rama website"), wxEmptyString, wxITEM_NORMAL);
    itemMenu41->Append(ID_MENU_HELP_LUA_MANUAL, _("Lua manual"), wxEmptyString, wxITEM_NORMAL);
    itemMenu41->Append(ID_MENU_HELP_INSTALL_LATEST, _("Install latest version..."), wxEmptyString, wxITEM_NORMAL);
    itemMenu41->Append(ID_MENU_HELP_ABOUT, _("&About..."), wxEmptyString, wxITEM_NORMAL);
    menuBar->Append(itemMenu41, _("&Help"));
    itemFrame1->SetMenuBar(menuBar);

    wxBoxSizer* itemBoxSizer47 = new wxBoxSizer(wxVERTICAL);
    itemFrame1->SetSizer(itemBoxSizer47);

    wxSplitterWindow* itemSplitterWindow48 = new wxSplitterWindow( itemFrame1, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxSP_LIVE_UPDATE|wxNO_BORDER|wxCLIP_CHILDREN );
    itemSplitterWindow48->SetMinimumPaneSize(50);

    wxPanel* itemPanel49 = new wxPanel( itemSplitterWindow48, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxSUNKEN_BORDER|wxCLIP_CHILDREN|wxTAB_TRAVERSAL );
    wxBoxSizer* itemBoxSizer50 = new wxBoxSizer(wxVERTICAL);
    itemPanel49->SetSizer(itemBoxSizer50);

    wxBoxSizer* itemBoxSizer51 = new wxBoxSizer(wxHORIZONTAL);
    itemBoxSizer50->Add(itemBoxSizer51, 0, wxGROW, 5);
    wxStaticText* itemStaticText52 = new wxStaticText( itemPanel49, wxID_STATIC, _("Current model:"), wxDefaultPosition, wxDefaultSize, 0 );
    itemBoxSizer51->Add(itemStaticText52, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5);

    script_filename_label_ = new wxStaticText( itemPanel49, wxID_STATIC, _("None"), wxDefaultPosition, wxDefaultSize, wxST_NO_AUTORESIZE );
    itemBoxSizer51->Add(script_filename_label_, 1, wxALIGN_CENTER_VERTICAL, 5);

    wxCheckBox* itemCheckBox54 = new wxCheckBox( itemPanel49, ID_CHECKBOX_RESET_PARAMS, _("Reload resets parameters to script defaults"), wxDefaultPosition, wxDefaultSize, 0 );
    itemCheckBox54->SetValue(true);
    itemBoxSizer50->Add(itemCheckBox54, 0, wxALIGN_LEFT|wxLEFT, 5);

    wxStaticLine* itemStaticLine55 = new wxStaticLine( itemPanel49, wxID_STATIC, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL );
    itemBoxSizer50->Add(itemStaticLine55, 0, wxGROW|wxALL, 5);

    parameter_scrolledwindow_ = new wxScrolledWindow( itemPanel49, wxID_ANY, wxDefaultPosition, wxSize(100, 100), wxNO_BORDER|wxVSCROLL );
    itemBoxSizer50->Add(parameter_scrolledwindow_, 1, wxGROW|wxLEFT, 5);
    parameter_scrolledwindow_->SetScrollbars(1, 1, 0, 0);
    parameter_flexgrid_ = new wxFlexGridSizer(0, 3, 0, 0);
    parameter_scrolledwindow_->SetSizer(parameter_flexgrid_);

    parameter_flexgrid_->AddGrowableCol(2);

    parameter_scrolledwindow_->FitInside();

    wxPanel* itemPanel58 = new wxPanel( itemSplitterWindow48, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxSUNKEN_BORDER|wxCLIP_CHILDREN|wxTAB_TRAVERSAL );
    wxBoxSizer* itemBoxSizer59 = new wxBoxSizer(wxVERTICAL);
    itemPanel58->SetSizer(itemBoxSizer59);

    aui_notebook_ = new wxAuiNotebook( itemPanel58, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxAUI_NB_TOP|wxAUI_NB_TAB_SPLIT|wxAUI_NB_TAB_MOVE|wxAUI_NB_WINDOWLIST_BUTTON|wxNO_BORDER|wxCLIP_CHILDREN );

    wxPanel* itemPanel61 = new wxPanel( aui_notebook_, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxNO_BORDER|wxTAB_TRAVERSAL );
    wxBoxSizer* itemBoxSizer62 = new wxBoxSizer(wxVERTICAL);
    itemPanel61->SetSizer(itemBoxSizer62);

    display_controls_sizer_ = new wxBoxSizer(wxHORIZONTAL);
    itemBoxSizer62->Add(display_controls_sizer_, 0, wxGROW, 5);
    wxArrayString itemChoice64Strings;
    itemChoice64Strings.Add(_("Hide mesh"));
    itemChoice64Strings.Add(_("Show mesh"));
    itemChoice64Strings.Add(_("Dielectric Re"));
    itemChoice64Strings.Add(_("Dielectric Im"));
    itemChoice64Strings.Add(_("Dielectric Abs"));
    wxChoice* itemChoice64 = new wxChoice( itemPanel61, ID_CHOICE_MESH, wxDefaultPosition, wxDefaultSize, itemChoice64Strings, 0 );
    itemChoice64->SetStringSelection(_("Hide mesh"));
    if (MainWin::ShowToolTips())
        itemChoice64->SetToolTip(_("Select how the solution mesh is displayed"));
    display_controls_sizer_->Add(itemChoice64, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5);

    view_field_checkbox_ = new wxCheckBox( itemPanel61, ID_CHECKBOX_FIELD, _("Fiel&d"), wxDefaultPosition, wxDefaultSize, 0 );
    view_field_checkbox_->SetValue(false);
    if (MainWin::ShowToolTips())
        view_field_checkbox_->SetToolTip(_("Toggle display of the field solution"));
    display_controls_sizer_->Add(view_field_checkbox_, 0, wxALIGN_CENTER_VERTICAL|wxRIGHT, 5);

    animate_checkbox_ = new wxCheckBox( itemPanel61, ID_CHECKBOX_ANIMATE, _("&Animate"), wxDefaultPosition, wxDefaultSize, 0 );
    animate_checkbox_->SetValue(false);
    if (MainWin::ShowToolTips())
        animate_checkbox_->SetToolTip(_("Toggle animation of the field solution"));
    display_controls_sizer_->Add(animate_checkbox_, 0, wxALIGN_CENTER_VERTICAL|wxRIGHT, 5);

    wxArrayString display_style_Ez_Strings;
    display_style_Ez_Strings.Add(_("Ez amplitude and phase"));
    display_style_Ez_Strings.Add(_("Ez max amplitude"));
    display_style_Ez_Strings.Add(_("H & Jsurf amplitude"));
    display_style_Ez_Strings.Add(_("Vector Jsurf"));
    display_style_Ez_Strings.Add(_("Vector H"));
    display_style_Ez_Strings.Add(_("Poynting vector"));
    display_style_Ez_ = new wxChoice( itemPanel61, ID_SELECTION_DISPLAY_STYLE, wxDefaultPosition, wxDefaultSize, display_style_Ez_Strings, 0 );
    display_style_Ez_->SetStringSelection(_("Ez amplitude and phase"));
    display_controls_sizer_->Add(display_style_Ez_, 0, wxALIGN_CENTER_VERTICAL|wxRIGHT, 5);

    wxArrayString display_style_Exy_Strings;
    display_style_Exy_Strings.Add(_("Hz amplitude and phase"));
    display_style_Exy_Strings.Add(_("Hz max amplitude"));
    display_style_Exy_Strings.Add(_("[Ex,Ey] max amplitude"));
    display_style_Exy_Strings.Add(_("Rotated vector E"));
    display_style_Exy_Strings.Add(_("Vector E, Jsurf"));
    display_style_Exy_Strings.Add(_("Poynting vector"));
    display_style_Exy_ = new wxChoice( itemPanel61, ID_SELECTION_DISPLAY_STYLE, wxDefaultPosition, wxDefaultSize, display_style_Exy_Strings, 0 );
    display_style_Exy_->SetStringSelection(_("Hz amplitude and phase"));
    display_style_Exy_->Show(false);
    display_controls_sizer_->Add(display_style_Exy_, 0, wxALIGN_CENTER_VERTICAL|wxRIGHT, 5);

    wxArrayString display_style_TM_Strings;
    display_style_TM_Strings.Add(_("Ez"));
    display_style_TM_Strings.Add(_("Ez amplitude"));
    display_style_TM_Strings.Add(_("[Ex,Ey] amplitude"));
    display_style_TM_Strings.Add(_("[Ex,Ey]"));
    display_style_TM_Strings.Add(_("[Hx,Hy]"));
    display_style_TM_ = new wxChoice( itemPanel61, ID_SELECTION_DISPLAY_STYLE, wxDefaultPosition, wxDefaultSize, display_style_TM_Strings, 0 );
    display_style_TM_->SetStringSelection(_("Ez"));
    display_style_TM_->Show(false);
    display_controls_sizer_->Add(display_style_TM_, 0, wxALIGN_CENTER_VERTICAL|wxRIGHT, 5);

    wxArrayString display_style_TE_Strings;
    display_style_TE_Strings.Add(_("Hz"));
    display_style_TE_Strings.Add(_("Hz amplitude"));
    display_style_TE_Strings.Add(_("[Hx,Hy] amplitude"));
    display_style_TE_Strings.Add(_("[Hx,Hy]"));
    display_style_TE_Strings.Add(_("[Ex,Ey]"));
    display_style_TE_ = new wxChoice( itemPanel61, ID_SELECTION_DISPLAY_STYLE, wxDefaultPosition, wxDefaultSize, display_style_TE_Strings, 0 );
    display_style_TE_->SetStringSelection(_("Hz"));
    display_style_TE_->Show(false);
    display_controls_sizer_->Add(display_style_TE_, 0, wxALIGN_CENTER_VERTICAL|wxRIGHT, 5);

    mode_number_label_ = new wxStaticText( itemPanel61, wxID_STATIC, _("Mode"), wxDefaultPosition, wxDefaultSize, 0 );
    mode_number_label_->Show(false);
    display_controls_sizer_->Add(mode_number_label_, 0, wxALIGN_CENTER_VERTICAL|wxLEFT, 5);

    mode_number_ = new wxSpinCtrl( itemPanel61, ID_SPIN_MODE_NUMBER, wxT("0"), wxDefaultPosition, wxSize(80, -1), wxSP_ARROW_KEYS, 0, 1000, 0 );
    mode_number_->Show(false);
    display_controls_sizer_->Add(mode_number_, 0, wxALIGN_CENTER_VERTICAL|wxRIGHT, 5);

    wxArrayString itemChoice73Strings;
    itemChoice73Strings.Add(_("Color Jet"));
    itemChoice73Strings.Add(_("Color Hot"));
    itemChoice73Strings.Add(_("Color Gray"));
    itemChoice73Strings.Add(_("Color HSV"));
    itemChoice73Strings.Add(_("Color Bone"));
    itemChoice73Strings.Add(_("Color Copper"));
    itemChoice73Strings.Add(_("Color Wheel"));
    wxChoice* itemChoice73 = new wxChoice( itemPanel61, ID_SELECTION_COLORMAP, wxDefaultPosition, wxDefaultSize, itemChoice73Strings, 0 );
    itemChoice73->SetStringSelection(_("Color Jet"));
    display_controls_sizer_->Add(itemChoice73, 0, wxALIGN_CENTER_VERTICAL|wxRIGHT, 5);

    wxStaticText* itemStaticText74 = new wxStaticText( itemPanel61, wxID_STATIC, _("Brightness:"), wxDefaultPosition, wxDefaultSize, 0 );
    display_controls_sizer_->Add(itemStaticText74, 0, wxALIGN_CENTER_VERTICAL, 5);

    wxSlider* itemSlider75 = new wxSlider( itemPanel61, ID_SLIDER_BRIGHTNESS, 500, 0, 1000, wxDefaultPosition, wxSize(150, -1), wxSL_HORIZONTAL );
    display_controls_sizer_->Add(itemSlider75, 0, wxALIGN_CENTER_VERTICAL, 5);

    wxStaticLine* itemStaticLine76 = new wxStaticLine( itemPanel61, wxID_STATIC, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL );
    itemBoxSizer62->Add(itemStaticLine76, 0, wxGROW|wxTOP, 1);

    viewer_ = new Cavity( itemPanel61, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxNO_BORDER|wxCLIP_CHILDREN|wxTAB_TRAVERSAL );
    viewer_->SetBackgroundColour(wxColour(0, 0, 255));
    itemBoxSizer62->Add(viewer_, 1, wxGROW, 5);

    aui_notebook_->AddPage(itemPanel61, _("Model"), false);

    plot_ = new wxPlot( aui_notebook_, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxNO_BORDER|wxCLIP_CHILDREN|wxTAB_TRAVERSAL );
    wxBoxSizer* itemBoxSizer79 = new wxBoxSizer(wxVERTICAL);
    plot_->SetSizer(itemBoxSizer79);

    wxBoxSizer* itemBoxSizer80 = new wxBoxSizer(wxHORIZONTAL);
    itemBoxSizer79->Add(itemBoxSizer80, 0, wxGROW|wxLEFT|wxTOP, 5);
    wxArrayString plot_showing_Strings;
    plot_showing_Strings.Add(_("Magnitude"));
    plot_showing_Strings.Add(_("Phase"));
    plot_showing_Strings.Add(_("Group delay"));
    plot_showing_ = new wxChoice( plot_, ID_SELECT_PLOT, wxDefaultPosition, wxDefaultSize, plot_showing_Strings, 0 );
    plot_showing_->SetStringSelection(_("Magnitude"));
    itemBoxSizer80->Add(plot_showing_, 0, wxALIGN_CENTER_VERTICAL, 5);

    aui_notebook_->AddPage(plot_, _("Plots"), false);

    antenna_pattern_ = new wxPlot( aui_notebook_, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxNO_BORDER|wxCLIP_CHILDREN|wxTAB_TRAVERSAL );
    wxBoxSizer* itemBoxSizer83 = new wxBoxSizer(wxVERTICAL);
    antenna_pattern_->SetSizer(itemBoxSizer83);

    wxBoxSizer* itemBoxSizer84 = new wxBoxSizer(wxHORIZONTAL);
    itemBoxSizer83->Add(itemBoxSizer84, 0, wxGROW|wxLEFT|wxTOP, 5);
    wxCheckBox* itemCheckBox85 = new wxCheckBox( antenna_pattern_, ID_ANTENNA_SHOW, _("Show"), wxDefaultPosition, wxDefaultSize, 0 );
    itemCheckBox85->SetValue(false);
    itemCheckBox85->SetBackgroundColour(wxColour(204, 204, 204));
    itemBoxSizer84->Add(itemCheckBox85, 0, wxALIGN_CENTER_VERTICAL, 5);

    wxCheckBox* itemCheckBox86 = new wxCheckBox( antenna_pattern_, ID_ANTENNA_SCALE_MAX, _("Scale max to 0dB"), wxDefaultPosition, wxDefaultSize, 0 );
    itemCheckBox86->SetValue(false);
    itemCheckBox86->SetBackgroundColour(wxColour(204, 204, 204));
    itemBoxSizer84->Add(itemCheckBox86, 0, wxALIGN_CENTER_VERTICAL|wxLEFT, 5);

    aui_notebook_->AddPage(antenna_pattern_, _("Antenna"), false);

    script_messages_ = new wxListCtrl( aui_notebook_, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLC_REPORT|wxLC_NO_HEADER|wxNO_BORDER );

    aui_notebook_->AddPage(script_messages_, _("Script messages"), false);

    itemBoxSizer59->Add(aui_notebook_, 1, wxGROW, 5);

    itemSplitterWindow48->SplitVertically(itemPanel49, itemPanel58, 300);
    itemBoxSizer47->Add(itemSplitterWindow48, 1, wxGROW, 5);

////@end MainWin content construction

    // Connect the viewer to other controls.
    viewer_->Connect(script_messages_, aui_notebook_, parameter_flexgrid_, parameter_scrolledwindow_, plot_);
    viewer_->Connect2(antenna_pattern_);

    // Configure the script messages control.
    script_messages_->InsertColumn(0, "Message");

    // Create an image list for the script messages control. The images are
    //   0: blank
    //   1: info
    //   2: warning
    //   3: error
    static const int ICON_SIZE = 16;
    wxImageList *imageList = new wxImageList(ICON_SIZE, ICON_SIZE);
    wxImage white_icon = wxImage(ICON_SIZE, ICON_SIZE);
    white_icon.SetRGB(wxRect(0, 0, ICON_SIZE, ICON_SIZE), 255, 255, 255);
    imageList->Add(wxBitmap(white_icon));
    imageList->Add(wxArtProvider::GetBitmap(wxART_INFORMATION, wxART_MESSAGE_BOX, wxSize(ICON_SIZE, ICON_SIZE)));
    imageList->Add(wxArtProvider::GetBitmap(wxART_WARNING, wxART_MESSAGE_BOX, wxSize(ICON_SIZE, ICON_SIZE)));
    imageList->Add(wxArtProvider::GetBitmap(wxART_ERROR, wxART_MESSAGE_BOX, wxSize(ICON_SIZE, ICON_SIZE)));
    script_messages_->AssignImageList(imageList, wxIMAGE_LIST_SMALL);
}


/*
 * Should we show tooltips?
 */

bool MainWin::ShowToolTips()
{
    return true;
}

/*
 * Get bitmap resources
 */

wxBitmap MainWin::GetBitmapResource( const wxString& name )
{
    // Bitmap retrieval
////@begin MainWin bitmap retrieval
    wxUnusedVar(name);
    return wxNullBitmap;
////@end MainWin bitmap retrieval
}

/*
 * Get icon resources
 */

wxIcon MainWin::GetIconResource( const wxString& name )
{
    // Icon retrieval
////@begin MainWin icon retrieval
    wxUnusedVar(name);
    return wxNullIcon;
////@end MainWin icon retrieval
}

void MainWin::DoInitializationThatRequiresAnEventLoop() {
  // On GTK the wxFileSystemWatcher needs an active event loop before it can be
  // initialized.
  model_watcher_ = new wxFileSystemWatcher;
  model_watcher_->SetOwner(this);
}

void MainWin::DoShutdownThatRequiresAnEventLoop() {
  // On GTK the wxFileSystemWatcher must be shut down before the event loop is
  // shut down.
  delete model_watcher_;
  model_watcher_ = 0;
}

void MainWin::OnMenuOpen( wxCommandEvent& event ) {
    wxFileDialog dlg(this, "Select a model file to open", "", "", "*.lua", wxFD_OPEN | wxFD_FILE_MUST_EXIST | wxFD_CHANGE_DIR);
    if (dlg.ShowModal() == wxID_OK) {
        LoadFile(dlg.GetPath());
    }
}

void MainWin::LoadFile(const wxString &full_path) {
    model_watcher_->RemoveAll();
    script_filename_.Assign(full_path);
    script_filename_.MakeAbsolute();
    if (ReloadScript(true)) {
        // Watch this directory (on windows) or file (on linux,mac) for changes.
        #ifdef __WXMSW__
            model_watcher_->Add(script_filename_.GetPath(), wxFSW_EVENT_MODIFY);
        #else
            model_watcher_->Add(script_filename_.GetFullPath(), wxFSW_EVENT_MODIFY);
        #endif
    }
}

void MainWin::OnCurrentModelLabelTimeout(wxTimerEvent& event) {
    script_filename_label_->SetBackgroundColour(wxNullColour);
    script_filename_label_->Refresh();
}

void MainWin::OnMenuExit( wxCommandEvent& event ) {
    Close();
}

void MainWin::OnMenuReload( wxCommandEvent& event ) {
    ReloadScript(true);
}

void MainWin::OnModelFileChanged(wxFileSystemWatcherEvent& event) {
    // On MS Windows we can only monitor directories and we get this event more than
    // once for a file modification (it depends on how the editor saves the file).
    // To try and avoid reading the file in some in-between state, we trigger the
    // actual file read 100ms from now.
    reload_timer_.StartOnce(100);
}

void MainWin::OnReloadTimeout(wxTimerEvent& event) {
    if (autorun_) {
        ReloadScript(false);
    }
}

bool MainWin::ReloadScript(bool rerun_even_if_same) {
    if (!script_filename_.IsOk()) {
        wxLogError("No script has been opened yet");
        return false;
    }

    // Open and read the file.
    wxFile f;
    if (!f.Open(script_filename_.GetFullPath())) {
        wxLogError("Can not open %s", script_filename_.GetFullPath());
        return false;
    }
    wxString buffer;
    if (!f.ReadAll(&buffer)) {
        wxLogError("Can not read %s", script_filename_.GetFullPath());
        return false;
    }
    if (buffer.size() == 0) {
        // The file seems empty, it's possible we loaded it just as the
        // editor was saving it.
        wxLogError("File seems empty: %s", script_filename_.GetFullPath());
        return false;
    }

    // Run the script.
    bool script_ran = viewer_->RunScript(buffer.c_str(), rerun_even_if_same);

    // Highlight the "current model" label, revert it to normal color after 1s.
    if (script_ran) {
        script_filename_label_->SetBackgroundColour(wxColour(255, 255, 0));
        script_filename_label_->SetLabel(script_filename_.GetFullName());
        current_model_label_timer_.StartOnce(1000);
    }

    // Show the correct controls based on the cavity type.
    display_style_Ez_->Show(viewer_->IsEzCavity());
    display_style_Exy_->Show(viewer_->IsExyCavity());
    display_style_TE_->Show(viewer_->IsTEMode());
    display_style_TM_->Show(viewer_->IsTMMode());
    animate_checkbox_->Show(viewer_->NumWaveguideModes() == 0);
    mode_number_->Show(viewer_->NumWaveguideModes());
    mode_number_label_->Show(viewer_->NumWaveguideModes());
    mode_number_->SetRange(0, std::max(0, viewer_->NumWaveguideModes() - 1));
    viewer_->SetWaveguideModeDisplayed(mode_number_->GetValue());
    display_controls_sizer_->Layout();
    return true;
}

void MainWin::OnMenuAutorun( wxCommandEvent& event ) {
    autorun_ = !autorun_;
    if (autorun_) {
        ReloadScript(false);
    }
}

void MainWin::OnMenuViewZoomExtents( wxCommandEvent& event ) {
    viewer_->ZoomToExtents();
}

void MainWin::OnMenuToggleBoundaryVertices( wxCommandEvent& event ) {
    viewer_->ToggleShowBoundaryVertices();
}

void MainWin::OnMenuToggleBoundary( wxCommandEvent& event ) {
    viewer_->ToggleShowBoundary();
}

void MainWin::OnMenuToggleBoundaryDerivatives( wxCommandEvent& event ) {
    viewer_->ToggleShowBoundaryDerivatives();
}

void MainWin::OnMenuHelpAbout( wxCommandEvent& event ) {
	AboutWin win(this);
	win.ShowModal();
}

void MainWin::OnMenuToggleGrid( wxCommandEvent& event ) {
    viewer_->ToggleGrid();
}

void MainWin::OnMenuToggleAntialiasing( wxCommandEvent& event ) {
    viewer_->ToggleAntialiasing();
}

void MainWin::OnCheckboxResetParams( wxCommandEvent& event ) {
    viewer_->SetIfRunScriptResetsParameters(event.GetInt());
}

void MainWin::OnViewFieldClick( wxCommandEvent& event ) {
    // Turning off the field view automatically turns off the animation.
    if (!event.GetInt()) {
        if (animate_checkbox_->GetValue()) {
            animate_checkbox_->SetValue(0);
            viewer_->Animate(0);        // As wxEVT_CHECKBOX will not be emitted by SetValue()
        }
    }
    viewer_->ViewField(event.GetInt());
}

void MainWin::OnAnimateClick( wxCommandEvent& event ) {
    // Turning on the animation automatically turns on the field view.
    if (event.GetInt()) {
        if (!view_field_checkbox_->GetValue()) {
            view_field_checkbox_->SetValue(1);
            viewer_->ViewField(1);      // As wxEVT_CHECKBOX will not be emitted by SetValue()
        }
    }
    viewer_->Animate(event.GetInt());
}

void MainWin::OnDisplayStyleSelected( wxCommandEvent& event ) {
    display_style_Ez_->SetSelection(event.GetInt());
    display_style_Exy_->SetSelection(event.GetInt());
    viewer_->SetDisplayStyle(event.GetInt());
}

void MainWin::OnDisplayColorSelected( wxCommandEvent& event ) {
    viewer_->SetDisplayColorScheme(event.GetInt());
}

void MainWin::OnSliderBrightnessScroll( wxScrollEvent& event ) {
    viewer_->SetBrightness(event.GetInt());
}

void MainWin::OnMenuSolveSweep( wxCommandEvent& event ) {
    SweepDialog dlg(this, viewer_);
    if (dlg.ShowModal() == wxID_OK) {
        viewer_->Sweep(dlg.GetParameterName(),
                       dlg.GetStartValue(),
                       dlg.GetEndValue(),
                       dlg.GetNumSteps());
    }
}

void MainWin::OnMenuExportFieldMatlab( wxCommandEvent& event ) {
    wxFileDialog dlg(this, "Select a matlab file to save", "", "", "*.mat", wxFD_SAVE | wxFD_OVERWRITE_PROMPT | wxFD_CHANGE_DIR);
    if (dlg.ShowModal() == wxID_OK) {
        viewer_->ExportFieldMatlab(dlg.GetPath().c_str());
    }
}

void MainWin::OnMenuExportPlotMatlabClick(wxCommandEvent& event) {
    wxFileDialog dlg(this, "Select a matlab file to save", "", "", "*.mat", wxFD_SAVE | wxFD_OVERWRITE_PROMPT | wxFD_CHANGE_DIR);
    if (dlg.ShowModal() == wxID_OK) {
        viewer_->ExportPlotMatlab(dlg.GetPath().c_str());
    }
}

void MainWin::OnMenuOptimize( wxCommandEvent& event ) {
    viewer_->Optimize();
}

void MainWin::OnMenuStopSolve( wxCommandEvent& event ) {
    viewer_->StopSweepOrOptimize();
}

void MainWin::OnMenuExportDxf( wxCommandEvent& event ) {
    wxFileDialog dlg(this, "Select a DXF file to save", "", "", "*.dxf", wxFD_SAVE | wxFD_OVERWRITE_PROMPT | wxFD_CHANGE_DIR);
    if (dlg.ShowModal() == wxID_OK) {
        viewer_->ExportBoundaryDXF(dlg.GetPath().c_str());
    }
}

void MainWin::OnMenuExportXy( wxCommandEvent& event ) {
    wxFileDialog dlg(this, "Select a file to save", "", "", "*.*", wxFD_SAVE | wxFD_OVERWRITE_PROMPT | wxFD_CHANGE_DIR);
    if (dlg.ShowModal() == wxID_OK) {
        viewer_->ExportBoundaryXY(dlg.GetPath().c_str());
    }
}

void MainWin::OnMenuHelpManual( wxCommandEvent& event ) {
    wxString path = wxStandardPaths::Get().GetResourcesDir();
    wxURI url("file://" + path + "/rama.html");
    wxLaunchDefaultBrowser(url.BuildURI(), wxBROWSER_NEW_WINDOW);
}

void MainWin::OnMenuHelpWebsite( wxCommandEvent& event ) {
    wxLaunchDefaultBrowser(__APP_URL__, wxBROWSER_NEW_WINDOW);
}

void MainWin::OnMenuHelpLuaManual( wxCommandEvent& event ) {
    wxLaunchDefaultBrowser("http://www.lua.org/manual/5.3/", wxBROWSER_NEW_WINDOW);
}

void MainWin::OnSelectPlot( wxCommandEvent& event ) {
    viewer_->SelectPlot(event.GetInt());
}

void MainWin::OnMenuToggleEmitTraceReport( wxCommandEvent& event ) {
    viewer_->ToggleEmitTraceReport();
}

void MainWin::OnMenuOptimizerLevenbergMarquardt( wxCommandEvent& event ) {
    viewer_->SetOptimizer(LEVENBERG_MARQUARDT);
}

void MainWin::OnMenuOptimizerDogleg( wxCommandEvent& event ) {
    viewer_->SetOptimizer(SUBSPACE_DOGLEG);
}

void MainWin::OnZoomIn( wxCommandEvent& event ) {
    viewer_->ZoomIn();
}

void MainWin::OnZoomOut( wxCommandEvent& event ) {
    viewer_->ZoomOut();
}

void MainWin::OnMenuToggleMarkers( wxCommandEvent& event ) {
    viewer_->ToggleShowMarkers();
}

void MainWin::OnCopy( wxCommandEvent& event ) {
    viewer_->CopyParametersToClipboard();
}

void MainWin::OnChoiceMesh(wxCommandEvent& event) {
    viewer_->ViewMesh(event.GetInt());
}

void MainWin::OnMenuHelpInstallLatest(wxCommandEvent& event) {
    wxMessageDialog dlg (this, "Clicking 'Yes' will quit the application and start "
        "downloading the installer for the latest version of " __APP_NAME__
        ". Do you want to proceed?", "Install latest version?",
        wxYES_NO | wxNO_DEFAULT | wxCENTRE);
    if (dlg.ShowModal() == wxID_YES) {
        #if __APPLE__
        wxLaunchDefaultBrowser(__APP_URL__ __APP_LATEST_MAC_DOWNLOAD_PATH__, wxBROWSER_NEW_WINDOW);
        #else
        wxLaunchDefaultBrowser(__APP_URL__ __APP_LATEST_WINDOWS_DOWNLOAD_PATH__, wxBROWSER_NEW_WINDOW);
        #endif
        exit(0);
    }
}

void MainWin::OnAntennaShow(wxCommandEvent &event) {
    viewer_->ToggleAntennaShow();
}

void MainWin::OnAntennaScaleMax(wxCommandEvent &event) {
    viewer_->ToggleAntennaScaleMax();
}

void MainWin::OnSpinModeNumberUpdated(wxSpinEvent &event) {
    viewer_->SetWaveguideModeDisplayed(mode_number_->GetValue());
}

