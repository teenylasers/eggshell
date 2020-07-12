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

// Import static plugin classes for static plugins.

#include <QtPlugin>

#ifdef __APPLE__
  Q_IMPORT_PLUGIN(QCocoaIntegrationPlugin)
  Q_IMPORT_PLUGIN(QMacStylePlugin)
#endif

#ifdef __WINNT__
  Q_IMPORT_PLUGIN(QWindowsIntegrationPlugin)
#endif

#ifdef __linux__
  Q_IMPORT_PLUGIN(QXcbIntegrationPlugin)
  Q_IMPORT_PLUGIN(QXcbGlxIntegrationPlugin)
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
