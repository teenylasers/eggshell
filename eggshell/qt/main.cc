#include "main_window.h"
#include <QApplication>
#include <QSurfaceFormat>
#include "../../toolkit/gl_utils.h"

int main(int argc, char *argv[]) {
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

  // Setup the qt error handler. This must be done after the QApplication is
  // constructed.
  SetErrorHandler(new qtErrorHandler);

  // Run the app.
  MainWindow w;
  w.show();
  return app.exec();
}
