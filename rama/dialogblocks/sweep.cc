/////////////////////////////////////////////////////////////////////////////
// Name:        sweep.cc
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

////@begin includes
////@end includes

#include "sweep.h"
#include "../cavity.h"
#include "../../toolkit/mystring.h"

////@begin XPM images
////@end XPM images


/*
 * SweepDialog type definition
 */

IMPLEMENT_DYNAMIC_CLASS( SweepDialog, wxDialog )


/*
 * SweepDialog event table definition
 */

BEGIN_EVENT_TABLE( SweepDialog, wxDialog )

////@begin SweepDialog event table entries
    EVT_CHOICE( ID_SWEEP_PARAMETER, SweepDialog::OnSweepParameterSelected )
    EVT_BUTTON( wxID_OK, SweepDialog::OnOk )
////@end SweepDialog event table entries

END_EVENT_TABLE()


/*
 * SweepDialog constructors
 */

SweepDialog::SweepDialog()
{
    Init();
}

SweepDialog::SweepDialog( wxWindow* parent, Cavity* viewer, wxWindowID id, const wxString& caption, const wxPoint& pos, const wxSize& size, long style )
{
    Init();
    viewer_ = viewer;
    Create(parent, id, caption, pos, size, style);
}


/*
 * Sweep creator
 */

bool SweepDialog::Create( wxWindow* parent, wxWindowID id, const wxString& caption, const wxPoint& pos, const wxSize& size, long style )
{
////@begin SweepDialog creation
    SetExtraStyle(wxWS_EX_BLOCK_EVENTS);
    wxDialog::Create( parent, id, caption, pos, size, style );

    CreateControls();
    if (GetSizer())
    {
        GetSizer()->SetSizeHints(this);
    }
    Centre();
////@end SweepDialog creation
    return true;
}


/*
 * SweepDialog destructor
 */

SweepDialog::~SweepDialog()
{
////@begin SweepDialog destruction
////@end SweepDialog destruction
}


/*
 * Member initialisation
 */

void SweepDialog::Init()
{
////@begin SweepDialog member initialisation
    parameters_ctrl_ = NULL;
    start_value_ctrl_ = NULL;
    end_value_ctrl_ = NULL;
    num_steps_text_ = NULL;
    num_steps_ctrl_ = NULL;
////@end SweepDialog member initialisation
    viewer_ = NULL;
    integer_ = false;
    start_value_ = end_value_ = min_value_ = max_value_ = 0;
    num_steps_ = 0;
}


/*
 * Control creation for Sweep
 */

void SweepDialog::CreateControls()
{
////@begin SweepDialog content construction
    SweepDialog* itemDialog1 = this;

    wxBoxSizer* itemBoxSizer2 = new wxBoxSizer(wxVERTICAL);
    itemDialog1->SetSizer(itemBoxSizer2);

    wxFlexGridSizer* itemFlexGridSizer3 = new wxFlexGridSizer(0, 2, 0, 0);
    itemBoxSizer2->Add(itemFlexGridSizer3, 0, wxALIGN_CENTER_HORIZONTAL|wxALL, 5);

    wxStaticText* itemStaticText4 = new wxStaticText( itemDialog1, wxID_STATIC, _("Parameter to sweep"), wxDefaultPosition, wxDefaultSize, 0 );
    itemFlexGridSizer3->Add(itemStaticText4, 0, wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL|wxALL, 5);

    wxArrayString parameters_ctrl_Strings;
    parameters_ctrl_ = new wxChoice( itemDialog1, ID_SWEEP_PARAMETER, wxDefaultPosition, wxDefaultSize, parameters_ctrl_Strings, 0 );
    itemFlexGridSizer3->Add(parameters_ctrl_, 0, wxGROW|wxALIGN_CENTER_VERTICAL|wxALL, 5);

    wxStaticText* itemStaticText6 = new wxStaticText( itemDialog1, wxID_STATIC, _("Start value"), wxDefaultPosition, wxDefaultSize, 0 );
    itemFlexGridSizer3->Add(itemStaticText6, 0, wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL|wxALL, 5);

    start_value_ctrl_ = new wxTextCtrl( itemDialog1, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
    itemFlexGridSizer3->Add(start_value_ctrl_, 0, wxGROW|wxALIGN_CENTER_VERTICAL|wxALL, 5);

    wxStaticText* itemStaticText8 = new wxStaticText( itemDialog1, wxID_STATIC, _("End value"), wxDefaultPosition, wxDefaultSize, 0 );
    itemFlexGridSizer3->Add(itemStaticText8, 0, wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL|wxALL, 5);

    end_value_ctrl_ = new wxTextCtrl( itemDialog1, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
    itemFlexGridSizer3->Add(end_value_ctrl_, 0, wxGROW|wxALIGN_CENTER_VERTICAL|wxALL, 5);

    num_steps_text_ = new wxStaticText( itemDialog1, wxID_STATIC, _("Number of steps"), wxDefaultPosition, wxDefaultSize, 0 );
    itemFlexGridSizer3->Add(num_steps_text_, 0, wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL|wxALL, 5);

    num_steps_ctrl_ = new wxTextCtrl( itemDialog1, wxID_ANY, _("50"), wxDefaultPosition, wxDefaultSize, 0 );
    itemFlexGridSizer3->Add(num_steps_ctrl_, 0, wxGROW|wxALIGN_CENTER_VERTICAL|wxALL, 5);

    wxStaticLine* itemStaticLine12 = new wxStaticLine( itemDialog1, wxID_STATIC, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL );
    itemBoxSizer2->Add(itemStaticLine12, 0, wxGROW|wxALL, 5);

    wxBoxSizer* itemBoxSizer13 = new wxBoxSizer(wxHORIZONTAL);
    itemBoxSizer2->Add(itemBoxSizer13, 0, wxALIGN_CENTER_HORIZONTAL|wxALL, 5);

    wxButton* itemButton14 = new wxButton( itemDialog1, wxID_OK, _("OK"), wxDefaultPosition, wxDefaultSize, 0 );
    itemButton14->SetDefault();
    itemBoxSizer13->Add(itemButton14, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5);

    wxButton* itemButton15 = new wxButton( itemDialog1, wxID_CANCEL, _("Cancel"), wxDefaultPosition, wxDefaultSize, 0 );
    itemBoxSizer13->Add(itemButton15, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5);

////@end SweepDialog content construction

    std::vector<std::string> names;
    viewer_->GetParamControlNames(&names);
    for (int i = 0; i < names.size(); i++) {
        parameters_ctrl_->Append(names[i]);
    }
    parameters_ctrl_->SetSelection(0);
    if (!names.empty()) {
        SetParameter(names[0]);
    }
}


/*
 * Should we show tooltips?
 */

bool SweepDialog::ShowToolTips()
{
    return true;
}

/*
 * Get bitmap resources
 */

wxBitmap SweepDialog::GetBitmapResource( const wxString& name )
{
    // Bitmap retrieval
////@begin SweepDialog bitmap retrieval
    wxUnusedVar(name);
    return wxNullBitmap;
////@end SweepDialog bitmap retrieval
}

/*
 * Get icon resources
 */

wxIcon SweepDialog::GetIconResource( const wxString& name )
{
    // Icon retrieval
////@begin SweepDialog icon retrieval
    wxUnusedVar(name);
    return wxNullIcon;
////@end SweepDialog icon retrieval
}


/*
 * wxEVT_COMMAND_CHOICE_SELECTED event handler for ID_SWEEP_PARAMETER
 */

void SweepDialog::OnSweepParameterSelected( wxCommandEvent& event ) {
    SetParameter((const char*) parameters_ctrl_->GetString(event.GetInt()).c_str());
}

void SweepDialog::SetParameter(const std::string &name) {
    parameter_ = name;
    const Parameter &param = viewer_->GetParameter(name);
    start_value_ctrl_->SetValue(wxString::Format("%.10g", param.the_min));
    end_value_ctrl_->SetValue(wxString::Format("%.10g", param.the_max));
    num_steps_ctrl_->Enable(!param.integer);
    num_steps_text_->Enable(!param.integer);
    integer_ = param.integer;
    min_value_ = param.the_min;
    max_value_ = param.the_max;
}

void SweepDialog::OnOk( wxCommandEvent& event ) {
    if (!StrToDouble(start_value_ctrl_->GetValue().c_str(), &start_value_)) {
        wxLogError("Bad start value");
        return;
    }
    if (!StrToDouble(end_value_ctrl_->GetValue().c_str(), &end_value_)) {
        wxLogError("Bad end value");
        return;
    }
    if (!StrToInt(num_steps_ctrl_->GetValue().c_str(), &num_steps_) ||
        num_steps_ <= 0) {
        wxLogError("Bad number of steps");
        return;
    }
    if (start_value_ >= end_value_) {
        wxLogError("Start value must be less than end value");
        return;
    }
    if (start_value_ < min_value_ || end_value_ > max_value_) {
        wxLogError("Start and end must be in the range %g to %g", min_value_, max_value_);
        return;
    }
    if (integer_) {
        if (start_value_ != int(start_value_) || end_value_ != int(end_value_)) {
            wxLogError("Start and end values must be integers");
            return;
        }
    }
    event.Skip();
}
