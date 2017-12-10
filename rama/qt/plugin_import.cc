// Import static plugin classes for static plugins.

#include <QtPlugin>

#ifdef __APPLE__
  Q_IMPORT_PLUGIN(QCocoaIntegrationPlugin)
#endif

#ifdef __WINNT__
  Q_IMPORT_PLUGIN(QWindowsIntegrationPlugin)
#endif

// Some others that might come in handy later:
//   Q_IMPORT_PLUGIN(QGifPlugin)
//   Q_IMPORT_PLUGIN(QICNSPlugin)
//   Q_IMPORT_PLUGIN(QICOPlugin)
//   Q_IMPORT_PLUGIN(QJpegPlugin)
//   Q_IMPORT_PLUGIN(QMacJp2Plugin)
//   Q_IMPORT_PLUGIN(QTgaPlugin)
//   Q_IMPORT_PLUGIN(QTiffPlugin)
//   Q_IMPORT_PLUGIN(QWbmpPlugin)
//   Q_IMPORT_PLUGIN(QWebpPlugin)
//   Q_IMPORT_PLUGIN(QCoreWlanEnginePlugin)
//   Q_IMPORT_PLUGIN(QGenericEnginePlugin)
