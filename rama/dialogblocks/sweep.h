/////////////////////////////////////////////////////////////////////////////
// Name:        sweep.h
// Purpose:
// Author:
// Modified by:
// Created:
// RCS-ID:
// Copyright:
// Licence:
/////////////////////////////////////////////////////////////////////////////

#ifndef _SWEEP_H_
#define _SWEEP_H_


/*!
 * Includes
 */

////@begin includes
#include "wx/statline.h"
////@end includes

/*!
 * Forward declarations
 */

class Cavity;
////@begin forward declarations
////@end forward declarations

/*!
 * Control identifiers
 */

////@begin control identifiers
#define ID_SWEEP_PARAMETER 10009
#define SYMBOL_SWEEPDIALOG_STYLE wxCAPTION|wxCLOSE_BOX|wxTAB_TRAVERSAL
#define SYMBOL_SWEEPDIALOG_TITLE _("Sweep parameters")
#define SYMBOL_SWEEPDIALOG_IDNAME wxID_ANY
#define SYMBOL_SWEEPDIALOG_SIZE wxSize(400, 300)
#define SYMBOL_SWEEPDIALOG_POSITION wxDefaultPosition
////@end control identifiers


/*!
 * SweepDialog class declaration
 */

class SweepDialog: public wxDialog
{
    DECLARE_DYNAMIC_CLASS( SweepDialog )
    DECLARE_EVENT_TABLE()

public:
    /// Constructors
    SweepDialog();
    SweepDialog( wxWindow* parent, Cavity* viewer, wxWindowID id = SYMBOL_SWEEPDIALOG_IDNAME, const wxString& caption = SYMBOL_SWEEPDIALOG_TITLE, const wxPoint& pos = SYMBOL_SWEEPDIALOG_POSITION, const wxSize& size = SYMBOL_SWEEPDIALOG_SIZE, long style = SYMBOL_SWEEPDIALOG_STYLE );

    /// Creation
    bool Create( wxWindow* parent, wxWindowID id = SYMBOL_SWEEPDIALOG_IDNAME, const wxString& caption = SYMBOL_SWEEPDIALOG_TITLE, const wxPoint& pos = SYMBOL_SWEEPDIALOG_POSITION, const wxSize& size = SYMBOL_SWEEPDIALOG_SIZE, long style = SYMBOL_SWEEPDIALOG_STYLE );

    /// Destructor
    ~SweepDialog();

    /// Initialises member variables
    void Init();

    /// Creates the controls and sizers
    void CreateControls();

////@begin SweepDialog event handler declarations

    /// wxEVT_COMMAND_CHOICE_SELECTED event handler for ID_SWEEP_PARAMETER
    void OnSweepParameterSelected( wxCommandEvent& event );

    /// wxEVT_COMMAND_BUTTON_CLICKED event handler for wxID_OK
    void OnOk( wxCommandEvent& event );

////@end SweepDialog event handler declarations

////@begin SweepDialog member function declarations

    /// Retrieves bitmap resources
    wxBitmap GetBitmapResource( const wxString& name );

    /// Retrieves icon resources
    wxIcon GetIconResource( const wxString& name );
////@end SweepDialog member function declarations

    /// Should we show tooltips?
    static bool ShowToolTips();

////@begin SweepDialog member variables
    wxChoice* parameters_ctrl_;
    wxTextCtrl* start_value_ctrl_;
    wxTextCtrl* end_value_ctrl_;
    wxStaticText* num_steps_text_;
    wxTextCtrl* num_steps_ctrl_;
////@end SweepDialog member variables
    Cavity *viewer_;
    std::string parameter_;
    bool integer_;
    double start_value_, end_value_, min_value_, max_value_;
    int num_steps_;

    void SetParameter(const std::string &name);
    std::string GetParameterName() const { return parameter_; }
    double GetStartValue() const { return start_value_; }
    double GetEndValue() const { return end_value_; }
    int GetNumSteps() const { return num_steps_; }
};

#endif
    // _SWEEP_H_
