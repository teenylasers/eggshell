/////////////////////////////////////////////////////////////////////////////
// Name:        aboutwin.h
// Purpose:
// Author:
// Modified by:
// Created:
// RCS-ID:
// Copyright:
// Licence:
/////////////////////////////////////////////////////////////////////////////

#ifndef _ABOUTWIN_H_
#define _ABOUTWIN_H_


/*!
 * Includes
 */

////@begin includes
#include "wx/statline.h"
////@end includes

/*!
 * Forward declarations
 */

////@begin forward declarations
////@end forward declarations

/*!
 * Control identifiers
 */

////@begin control identifiers
#define ID_ABOUTWIN 10000
#define SYMBOL_ABOUTWIN_STYLE wxCAPTION|wxCLOSE_BOX|wxTAB_TRAVERSAL
#define SYMBOL_ABOUTWIN_TITLE _("About")
#define SYMBOL_ABOUTWIN_IDNAME ID_ABOUTWIN
#define SYMBOL_ABOUTWIN_SIZE wxSize(400, 300)
#define SYMBOL_ABOUTWIN_POSITION wxDefaultPosition
////@end control identifiers


/*!
 * AboutWin class declaration
 */

class AboutWin: public wxDialog
{
    DECLARE_DYNAMIC_CLASS( AboutWin )
    DECLARE_EVENT_TABLE()

public:
    /// Constructors
    AboutWin();
    AboutWin( wxWindow* parent, wxWindowID id = SYMBOL_ABOUTWIN_IDNAME, const wxString& caption = SYMBOL_ABOUTWIN_TITLE, const wxPoint& pos = SYMBOL_ABOUTWIN_POSITION, const wxSize& size = SYMBOL_ABOUTWIN_SIZE, long style = SYMBOL_ABOUTWIN_STYLE );

    /// Creation
    bool Create( wxWindow* parent, wxWindowID id = SYMBOL_ABOUTWIN_IDNAME, const wxString& caption = SYMBOL_ABOUTWIN_TITLE, const wxPoint& pos = SYMBOL_ABOUTWIN_POSITION, const wxSize& size = SYMBOL_ABOUTWIN_SIZE, long style = SYMBOL_ABOUTWIN_STYLE );

    /// Destructor
    ~AboutWin();

    /// Initialises member variables
    void Init();

    /// Creates the controls and sizers
    void CreateControls();

////@begin AboutWin event handler declarations

////@end AboutWin event handler declarations

////@begin AboutWin member function declarations

    /// Retrieves bitmap resources
    wxBitmap GetBitmapResource( const wxString& name );

    /// Retrieves icon resources
    wxIcon GetIconResource( const wxString& name );
////@end AboutWin member function declarations

    /// Should we show tooltips?
    static bool ShowToolTips();

////@begin AboutWin member variables
    wxStaticText* app_name_;
    wxStaticText* version_;
    wxStaticText* build_date_;
////@end AboutWin member variables
};

#endif
    // _ABOUTWIN_H_
