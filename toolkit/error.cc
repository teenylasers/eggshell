
#ifdef __TOOLKIT_WXWINDOWS__
#include "stdwx.h"
#endif

#include "error.h"
#include "thread.h"
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "myvector"
#include "mystring.h"

using std::vector;

//***************************************************************************
// The basic error handling mechanism.

void Error(const char *msg, ...) {
  va_list ap;
  va_start(ap, msg);
  GetErrorHandler()->HandleError(ErrorHandler::Error, msg, ap);
}

void Warning(const char *msg, ...) {
  va_list ap;
  va_start(ap, msg);
  GetErrorHandler()->HandleError(ErrorHandler::Warning, msg, ap);
}

void Message(const char *msg, ...) {
  va_list ap;
  va_start(ap, msg);
  GetErrorHandler()->HandleError(ErrorHandler::Message, msg, ap);
}

void Panic(const char *msg, ...) {
  va_list ap;
  va_start(ap, msg);
  GetErrorHandler()->HandleError(ErrorHandler::Panic, msg, ap);
  _exit(1);
}

void VError(const char *msg, va_list ap) {
  GetErrorHandler()->HandleError(ErrorHandler::Error, msg, ap);
}

void VWarning(const char *msg, va_list ap) {
  GetErrorHandler()->HandleError(ErrorHandler::Warning, msg, ap);
}

void VMessage(const char *msg, va_list ap) {
  GetErrorHandler()->HandleError(ErrorHandler::Message, msg, ap);
}

void VPanic(const char *msg, va_list ap) {
  GetErrorHandler()->HandleError(ErrorHandler::Panic, msg, ap);
  _exit(1);
}

ErrorHandler::~ErrorHandler() {
}

struct DefaultErrorHandler : public ErrorHandler {
  void HandleError(Type type, const char *msg, va_list ap);
};

void DefaultErrorHandler::HandleError(Type type, const char *msg, va_list ap) {
  fflush(stdout);
  fflush(stderr);
  if (type == ErrorHandler::Error) {
    fprintf(stderr, "Error: ");
  } else if (type == ErrorHandler::Warning) {
    fprintf(stderr, "Warning: ");
  } else if (type == ErrorHandler::Message) {
    fprintf(stderr, "Message: ");
  } else if (type == ErrorHandler::Panic) {
    fprintf(stderr, "Panic: ");
  } else {
    fprintf(stderr, "<unknown error type>: ");
  }
  vfprintf(stderr, msg, ap);
  fprintf(stderr, "\n");
  fflush(stderr);
  if (type == ErrorHandler::Panic) {
    // Call _exit() here, which immediately terminates the process, rather than
    // exit() which will do all kinds of housekeeping such as running global
    // destructors, closing files and calling atexit functions. The reason is
    // that the system might be in a bad state (we paniced after all) and doing
    // that housekeeping could result in bad stuff happening, e.g. infinite
    // recursions.
    _exit(1);
  }
}

static DefaultErrorHandler default_error_handler;
static ErrorHandler *error_handler = &default_error_handler;
static Mutex error_handler_mutex;

ErrorHandler *SetErrorHandler(ErrorHandler *e) {
  MutexLock lock(&error_handler_mutex);
  ErrorHandler *ret = error_handler;
  error_handler = e;
  return ret;
}

ErrorHandler *GetErrorHandler() {
  MutexLock lock(&error_handler_mutex);
  return error_handler;
}

//***************************************************************************
// Complain-once mechanism.

static vector<bool*> complaint_flags;
static Mutex complaint_flags_mutex;

bool ComplaintFlagToReset(bool *flag) {
  MutexLock lock(&complaint_flags_mutex);
  if (!*flag) {
    complaint_flags.push_back(flag);
    *flag = true;
    return true;
  } else {
    return false;
  }
}

void ResetComplaints() {
  MutexLock lock(&complaint_flags_mutex);
  for (int i = 0; i < complaint_flags.size(); i++) {
    *complaint_flags[i] = false;
  }
  complaint_flags.clear();
}

//***************************************************************************
// wxWidgets error handling.

#ifdef __TOOLKIT_WXWINDOWS__

void wxErrorHandler::HandleError(ErrorHandler::Type type,
                                 const char *msg, va_list ap) {
  // Under windows the wxLog functions assume that %s strings are UTF-16, but
  // our own Error() functions assume byte-sized characters. Thus we need to
  // convert to wxString first.
  std::string buffer;
  {
    // We need separate copies of 'ap' for all users, as users will mutate it.
    va_list ap2;
    va_copy(ap2, ap);
    StringVPrintf(&buffer, msg, ap2);           //@@@@@@ declared in rem
  }
  wxString s = buffer;   // Convert string to wx native format

  if (type == ErrorHandler::Error) {
    wxLogError("%s", s);
  } else if (type == ErrorHandler::Warning) {
    wxLogWarning("%s", s);
  } else if (type == ErrorHandler::Message) {
    wxLogMessage("%s", s);
  } else if (type == ErrorHandler::Panic) {
    // On windows wxSafeShowMessage calls MessageBox, which runs its own event
    // loop and permits the main window to process a subset of events, such as
    // painting events. If this panic was triggered by a painting event,
    // recursing back into the painting code could potentially cause an endless
    // loop and all kinds of strange behavior. Therefore, we block all further
    // event processing now with an event filter.
    struct EventBlocker : public wxEventFilter {
      int FilterEvent(wxEvent &event) {
        return Event_Processed;
      }
    };
    EventBlocker blocker;
    wxEvtHandler::AddFilter(&blocker);

    // Emit the error message and exit the application.
    // @@@ Only under window does wxSafeShowMessage actually pop up a dialog
    //     box, on other platforms it just prints to stderr. Actually pop up
    //     a window on these other platforms.
    s += "\nApplication will now abort.";
    wxSafeShowMessage("Internal Error", s);
    default_error_handler.HandleError(type, msg, ap);
  }
}

#endif  // __TOOLKIT_WXWINDOWS__

//***************************************************************************
// qt error handling.

#ifdef QT_CORE_LIB

#include "qmessagebox.h"
#include "error_window.h"

void qtErrorHandler::HandleError(Type type, const char *msg, va_list ap) {
  // Create one error window and then use it forever.
  static ErrorWindow *errwin = 0;
  if (!errwin) {
    errwin = new ErrorWindow(0);
  }

  switch (type) {
    case ErrorHandler::Error:
      errwin->VAddLine(msg, ap, ErrorWindow::ERROR);
      break;
    case ErrorHandler::Warning:
      errwin->VAddLine(msg, ap, ErrorWindow::WARNING);
      break;
    case ErrorHandler::Message:
      errwin->VAddLine(msg, ap, ErrorWindow::MESSAGE);
      break;
    default: {
      // Print the error message to stderr first, in case we have lost the
      // ability to run the gui at this point. Then try to pop up a window,
      // then exit.
      char *s;
      vasprintf(&s, msg, ap);
      fprintf(stderr, "FATAL ERROR: ");
      fprintf(stderr, "%s\n", s);
      fflush(stdout);
      QMessageBox::critical(0, "Fatal error", s);
      // See the rationale in DefaultErrorHandler::HandleError() for this:
      _exit(1);
    }
  }
}

#endif  // QT_CORE_LIB
