// Version and application information that is shared by the app and the Inno
// Setup Compiler. Only preprocessor directives can go here.

#ifndef __VERSION_H__
#define __VERSION_H__

// We have separate numeric and string versions of the version number because
// the Inno Setup Compiler does not understand the full C preprocessor syntax
// and can not stringify the major and minor numbers.
#define __APP_VERSION_MAJOR__ 0
#define __APP_VERSION_MINOR__ 31
#define __APP_VERSION__ "0.31"
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

#endif
