## Building Qt for Windows

The Qt 5.9.2 distribution comes with precompiled libraries on most of the
platforms we care about. Qt on windows is more problematic. We want to use a
GNU (or equivalently, clang) toolchain so that we only have one compiler's
quirks to deal with everywhere (visual C++ would compile much faster on
windows but also require a lot of compiler-specific modifications to the
code). MinGW is a good option but the only MinGW libraries that are provided
are for 32 bit windows. Therfore we must build Qt on our own. We need a 64
bit GNU compiler that can use windows APIs and create standalone binaries.
The simplest option seems to be the `x86_64-w64-mingw32` toolchain from
cygwin.

Instructions:
* Install Qt source (installing [Qt Creator](https://www1.qt.io/download/) 
  is the simplest way).
* Install the prerequisites according to Qt's 
  [windows requirements](http://doc.qt.io/qt-5/windows-requirements.html) and 
  [windows building](http://doc.qt.io/qt-5/windows-building.html) instructions.
* Install [cygwin64](https://www.cygwin.com/install.html) and its 
  `x86_64-w64-mingw32` package.
* Install [msys2](http://www.msys2.org/) into its default location (`C:\msys64`).
* In `C:\msys64\usr\bin` run `cp make mingw32-make`.
* Install the msys2 `make` using `pacman -S make` from the msys2 command line, 
  or `c:\msys64\usr\bin\pacman -S make` from the windows command line.
* Add the following line to the source file
  `qtbase/mkspecs/win32-g++/qmake.conf` after the `include' line:
  ```
  CROSS_COMPILE=x86_64-w64-mingw32-
  ```
  This lets the build process find the mingw compiler. There are supposed to
  be Qt configure script options available that do the same thing but they
  don't appear to work consistently.
* Select a build and install directory that is different from the Qt source 
  directory (both should be empty initially).
* Download `qt5vars.cmd`. Modify the SET variables at the top to match your 
  configuration.
* Create a desktop shortcut to `qt5vars.cmd`. In the shortcut properties modify 
  the target to
  `%SystemRoot%\system32\cmd.exe /E:ON /V:ON /k C:\path_to_your\qt5vars.cmd`. 
  Modify the 'Start in' directory to your build directory.
* Run that shortcut to get a command prompt. Then run the following
  (changing the `-prefix` path to where your install directory is): 
  `..\Qt592_src\configure -prefix c:\path_to_install_directory -platform win32-g++ -shared -opensource -confirm-license -debug -no-icu -no-openssl -opengl desktop -no-angle -skip qtwebengine -skip qtlocation -nomake examples -nomake tests QMAKE_LFLAGS_CONSOLE+=-static-libstdc++`
* Run `make -j4`
* Run `make install`
