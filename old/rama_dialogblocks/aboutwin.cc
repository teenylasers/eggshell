/////////////////////////////////////////////////////////////////////////////
// Name:        aboutwin.cpp
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

#include "aboutwin.h"
#include "../version.h"

////@begin XPM images
////@end XPM images


/*
 * AboutWin type definition
 */

IMPLEMENT_DYNAMIC_CLASS( AboutWin, wxDialog )


/*
 * AboutWin event table definition
 */

BEGIN_EVENT_TABLE( AboutWin, wxDialog )

////@begin AboutWin event table entries
////@end AboutWin event table entries

END_EVENT_TABLE()


/*
 * AboutWin constructors
 */

AboutWin::AboutWin()
{
    Init();
}

AboutWin::AboutWin( wxWindow* parent, wxWindowID id, const wxString& caption, const wxPoint& pos, const wxSize& size, long style )
{
    Init();
    Create(parent, id, caption, pos, size, style);
}


/*
 * aboutwin creator
 */

bool AboutWin::Create( wxWindow* parent, wxWindowID id, const wxString& caption, const wxPoint& pos, const wxSize& size, long style )
{
////@begin AboutWin creation
    SetExtraStyle(wxWS_EX_BLOCK_EVENTS);
    wxDialog::Create( parent, id, caption, pos, size, style );

    CreateControls();
    if (GetSizer())
    {
        GetSizer()->SetSizeHints(this);
    }
    Centre();
////@end AboutWin creation
    return true;
}


/*
 * AboutWin destructor
 */

AboutWin::~AboutWin()
{
////@begin AboutWin destruction
////@end AboutWin destruction
}


/*
 * Member initialisation
 */

void AboutWin::Init()
{
////@begin AboutWin member initialisation
    app_name_ = NULL;
    version_ = NULL;
    build_date_ = NULL;
////@end AboutWin member initialisation
}


/*
 * Control creation for aboutwin
 */

void AboutWin::CreateControls()
{
////@begin AboutWin content construction
    AboutWin* itemDialog1 = this;

    wxBoxSizer* itemBoxSizer2 = new wxBoxSizer(wxVERTICAL);
    itemDialog1->SetSizer(itemBoxSizer2);

    app_name_ = new wxStaticText( itemDialog1, wxID_STATIC, _("AppName"), wxDefaultPosition, wxDefaultSize, 0 );
    app_name_->SetForegroundColour(wxColour(0, 128, 0));
    app_name_->SetFont(wxFont(20, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxT("Arial Black")));
    itemBoxSizer2->Add(app_name_, 0, wxALIGN_CENTER_HORIZONTAL|wxALL, 5);

    version_ = new wxStaticText( itemDialog1, wxID_STATIC, _("Version"), wxDefaultPosition, wxDefaultSize, 0 );
    itemBoxSizer2->Add(version_, 0, wxALIGN_CENTER_HORIZONTAL|wxALL, 5);

    build_date_ = new wxStaticText( itemDialog1, wxID_STATIC, _("Build date"), wxDefaultPosition, wxDefaultSize, 0 );
    itemBoxSizer2->Add(build_date_, 0, wxALIGN_CENTER_HORIZONTAL|wxALL, 5);

    wxStaticLine* itemStaticLine6 = new wxStaticLine( itemDialog1, wxID_STATIC, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL );
    itemBoxSizer2->Add(itemStaticLine6, 0, wxGROW|wxALL, 5);

    wxButton* itemButton7 = new wxButton( itemDialog1, wxID_OK, _("OK"), wxDefaultPosition, wxDefaultSize, 0 );
    itemBoxSizer2->Add(itemButton7, 0, wxALIGN_CENTER_HORIZONTAL|wxALL, 5);

////@end AboutWin content construction
    app_name_->SetLabelText(__APP_NAME__);
    version_->SetLabelText("Version " __APP_VERSION__);
    build_date_->SetLabelText(wxString::Format("Build date: %s, %s", __DATE__, __TIME__));
}


/*
 * Should we show tooltips?
 */

bool AboutWin::ShowToolTips()
{
    return true;
}

/*
 * Get bitmap resources
 */

wxBitmap AboutWin::GetBitmapResource( const wxString& name )
{
    // Bitmap retrieval
////@begin AboutWin bitmap retrieval
    wxUnusedVar(name);
    return wxNullBitmap;
////@end AboutWin bitmap retrieval
}

/*
 * Get icon resources
 */

wxIcon AboutWin::GetIconResource( const wxString& name )
{
    // Icon retrieval
////@begin AboutWin icon retrieval
    wxUnusedVar(name);
    return wxNullIcon;
////@end AboutWin icon retrieval
}
