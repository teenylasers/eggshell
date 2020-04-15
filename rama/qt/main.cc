#include "main_window.h"
#include "../../toolkit/error.h"
#include "../../toolkit/crash_handler.h"
#include "../../toolkit/testing.h"
#include "../../toolkit/mystring.h"
#include "../../toolkit/gl_utils.h"
#include "../version.h"
#include <QApplication>
#include <QSurfaceFormat>

int main(int argc, char *argv[]) {
  #if !defined(__WINNT__)
    SetupCrashHandling();
  #endif

  // If the -unittest flag is given on the command line, run all tests and exit.
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-unittest") == 0) {
      testing::RunAll();
      exit(0);
    }
  }

  // For convenient printf() debugging on windows and within the Qt Creator
  // console it is necessary to turn off stdout buffering. Neither environment
  // is line buffered.
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stderr, NULL, _IONBF, 0);

  // From the QOpenGLWidget class documentation:
  // Calling QSurfaceFormat::setDefaultFormat() before constructing the
  // QApplication instance is mandatory on some platforms (for example,
  // macOS) when an OpenGL core profile context is requested. This is to
  // ensure that resource sharing between contexts stays functional as all
  // internal contexts are created using the correct version and profile.
  gl::SetDefaultOpenGLSurfaceFormat();

  // Set some attributes, start the application. We need to share opengl
  // contexts because otherwise if we undock an QDockWidget containing an
  // QOpenGLWidget the GL context will be reinitialized, which will break
  // a bunch of initialize-once assumptions in e.g. gl::Shader. We *could*
  // implement QOpenGLWidget::initializeGL() to reinitialize everything,
  // but setting this attribute is easier.
  QApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
  QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
  QApplication app(argc, argv);
  #if defined(__WINNT__)
    app.setStyle("fusion");
  #endif

  // Setup the qt error handler. This must be done after the QApplication is
  // constructed.
  SetErrorHandler(new qtErrorHandler);

  // Start the main window. If a filename was specified on the command line,
  // load it. If the -test flag is given on the command line then run the
  // test() function in that file then exit.
  MainWindow w;
  w.SetCommandLineArguments(argc, argv);
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-test") == 0) {
      w.ToggleRunTestAfterSolve();
      w.ScriptTestMode();
      break;
    }
  }
  for (int i = 1; i < argc; i++) {
    if (argv[i][0] != '-') {
      w.LoadFile(argv[i]);
      break;
    }
  }
  w.show();

  return app.exec();
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
