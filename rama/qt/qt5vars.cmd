REM Set these paths according to your setup.
SET _MSYS=C:\msys64
SET _SRC=C:\path_to_qt_source
SET _BUILD=C:\path_to_qt_build
SET _CYGWIN_BIN=C:\cygwin64\bin
SET _MINGW_BIN=C:\cygwin64\usr\x86_64-w64-mingw32\sys-root\mingw\bin
SET _FLEXBISON=C:\path_to\win_flex_bison-2.5.5

REM Set the path. The MSYS bin directory must appear before anything else since
REM we need to pick up the windows filesystem compatible versions of make, sh
REM and other tools.
SET PATH=%_MSYS%\usr\bin;%_SRC%\qtbase\bin;%_BUILD%\qtbase\bin;%_CYGWIN_BIN%;%_MINGW_BIN%;%_FLEXBISON%;%PATH%

REM Set the shell to ingore CRs at the ends of windows text files.
SET SHELLOPTS=igncr

REM Pick up the correct compiler (necessary for bootstrapping qmake).
SET CXX=x86_64-w64-mingw32-g++
SET CC=x86_64-w64-mingw32-gcc

REM Erase temporary variables.
SET _MSYS=
SET _SRC=
SET _BUILD=
SET _CYGWIN_BIN=
SET _MINGW_BIN=
SET _FLEXBISON=
