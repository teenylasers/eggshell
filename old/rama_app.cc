// Rama Simulator, Copyright (C) 2014-2020 Russell Smith.
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

#include "stdwx.h"
#include <stdio.h>
#include "wx/protocol/http.h"
#include "dialogblocks/mainwin.h"
#include "common.h"
#include "crash_handler.h"
#include "testing.h"
#include "version.h"
#include "mystring.h"

// For wxWidgets on OS X, support for retina displays is dodgy and the API
// seems likely to be improved in >3.1. This makes sure we're using the
// expected version. When we upgrade wxWidgets we should carefully recheck
// rendering, see wxwidgets_patches.txt. E.g. make sure that Blit() in
// plot_wx.cc still works, see if wxOverlay (which is implemented in terms of
// Blit) is stable enough to replace SaveSnapshot() etc.

#if wxMAJOR_VERSION != 3 || wxMINOR_VERSION != 1 || wxRELEASE_NUMBER != 0
#error Unexpected version of wxWidgets.
#endif

//***************************************************************************
// Background thread to retrieve the latest application version number and
// display a message if there is a newer version available.

#ifdef __APPLE__
#define __APP_LATEST_VERSION_PATH__ __APP_LATEST_VERSION_PATH_MAC__
#else
#define __APP_LATEST_VERSION_PATH__ __APP_LATEST_VERSION_PATH_WIN__
#endif

class GetVersionNumberThread : public wxThread {
 public:
  GetVersionNumberThread();
  void *Entry();
};

GetVersionNumberThread::GetVersionNumberThread() : wxThread(wxTHREAD_DETACHED) {
}

void *GetVersionNumberThread::Entry() {
  // If any part of this fails then give up, and we'll try again the next time
  // the application is started.
  wxHTTP get;
  get.SetHeader(_T("Content-type"), _T("text/html; charset=utf-8"));
  get.SetTimeout(60);
  if (get.Connect(_T(__APP_URL_HOSTNAME__))) {
    wxInputStream *s =
        get.GetInputStream(_T(__APP_URL_PATH__ __APP_LATEST_VERSION_PATH__));
    if (s) {
      std::string version;
      int c = '0';
      while ((c >= '0' && c <= '9') || c == '.') {
        c = s->GetC();
        version += c;
      }
      delete s;
      int major, minor;
      if (sscanf(version.c_str(), "%d.%d", &major, &minor) == 2) {
        int latest = (major << 16) | minor;
        int current = (__APP_VERSION_MAJOR__ << 16) | __APP_VERSION_MINOR__;
        if (latest > current) {
          Message("A newer version of " __APP_NAME__ " is available (current "
                  "version is " __APP_VERSION__ ", latest version is %d.%d). "
                  "Use 'Help/Install Latest' to install the latest version.",
                  major, minor);
        }
      }
    }
  }

  // @@@ The wxSocketImplMac::DoClose bug happens before we get here, i.e.
  // before this thread quits.

  return 0;
}

//***************************************************************************
// Application class

class MyApp: public wxApp {
 public:
  MyApp();
  bool OnInit();
  void OnEventLoopEnter(wxEventLoopBase *loop);

 private:
  MainWin *main_win_;
  bool initialized_;
};

IMPLEMENT_APP(MyApp)

MyApp::MyApp() : main_win_(0), initialized_(false) {
  #ifndef __WXMSW__
    SetupCrashHandling();
  #endif

  // Don't log things more chatty than the Info level, even in wxWidgets debug
  // mode. This is necessary in the mac build because some of the trace
  // messages are malformed and cause a runtime error.
  wxLog::SetLogLevel(wxLOG_Info);

  // For convenient printf() debugging it is necessary to turn off stdout
  // buffering in the C runtime library on windows, as the only alternative is
  // full buffering which means that printf()s do not show up immediately.
  #ifdef __WXMSW__
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
  #endif
}

bool MyApp::OnInit() {
  // If the -test flag is given on the command line, run all tests and exit.
  for (int i = 1; i < argc; i++) {
    if (argv[i] == "-test") {
      testing::RunAll();
      exit(0);
    }
  }

  // Setup the wx error handler.
  SetErrorHandler(new wxErrorHandler);

  // Create the main window.
  main_win_ = new MainWin(0,-1,SYMBOL_MAINWIN_TITLE,
          SYMBOL_MAINWIN_POSITION,SYMBOL_MAINWIN_SIZE,SYMBOL_MAINWIN_STYLE);
  main_win_->Show(true);

  // We use idle events to do "invisible hand" processing. It's more efficient
  // if idle events are only handled by their target windows and not every
  // window. Note that the wxWS_EX_PROCESS_IDLE extra window style must be set
  // for every window that wants to receive idle events.
  wxIdleEvent::SetMode(wxIDLE_PROCESS_SPECIFIED);

  return true;
}

void MyApp::OnEventLoopEnter(wxEventLoopBase *loop) {
  // This is called multiple times in an application, not only at the start but
  // also e.g. when dialog boxes close. Make sure we only initialize things the
  // first time.
  if (!initialized_) {
    initialized_ = true;
    if (loop->IsMain()) {
      main_win_->DoInitializationThatRequiresAnEventLoop();

      // If a filename was specified on the command line, load it.
      for (int i = 1; i < argc; i++) {
        wxString s = argv[i];
        const char *arg = s.c_str();
        if (arg[0] != '-') {
          main_win_->LoadFile(arg);
          break;
        }
      }

      // Get the latest application version number and display a message if
      // there is a newer version available. The wxSocketBase::Initialize()
      // allows sockets to be used from multiple threads.
      if (wxSocketBase::Initialize()) {
        GetVersionNumberThread *v = new GetVersionNumberThread;
        v->Run();
      }
    }
  }
}

//***************************************************************************
// Tests.

TEST_FUNCTION(Version) {
  // Check the constants in version.h for consistency.
  std::string version;
  StringPrintf(&version, "%d.%d", __APP_VERSION_MAJOR__, __APP_VERSION_MINOR__);
  CHECK(version == __APP_VERSION__);
  std::string path = "http://";
  path += __APP_URL_HOSTNAME__;
  path += __APP_URL_PATH__;
  CHECK(path == __APP_URL__);
}
