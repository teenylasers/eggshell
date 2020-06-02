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

// Version and application information that is shared by the app and the Inno
// Setup Compiler. Only preprocessor directives can go here.

#ifndef __VERSION_H__
#define __VERSION_H__

// We have separate numeric and string versions of the version number because
// the Inno Setup Compiler does not understand the full C preprocessor syntax
// and can not stringify the major and minor numbers.
#define __APP_VERSION_MAJOR__ 0
#define __APP_VERSION_MINOR__ 34
// The followning line is grepped for by the Makefile, keep this format:
#define __APP_VERSION__ "0.34"
#define __APP_NAME__ "Rama"
#define __APP_EXE_NAME__ "rama.exe"
#define __APP_PUBLISHER__ ""
#define __APP_URL_HOSTNAME__ "ramasimulator.org"
#define __APP_URL_PATH__ ""
#define __APP_URL__ "http://ramasimulator.org"
#define __APP_LATEST_WINDOWS_DOWNLOAD_PATH__ "/setup_rama.exe"
#define __APP_LATEST_MAC_DOWNLOAD_PATH__ "/Rama.dmg"
#define __APP_LATEST_VERSION_PATH_WIN__ "/latest_version.txt"
#define __APP_LATEST_VERSION_PATH_MAC__ "/latest_version_mac.txt"
#define __APP_LATEST_VERSION_PATH_LINUX__ "/latest_version_linux.txt"

#endif
