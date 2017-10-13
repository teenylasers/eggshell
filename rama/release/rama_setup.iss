; Inno Setup Compiler Script.

#include "..\version.h"

[Setup]
; NOTE: The value of AppId uniquely identifies this application.
; Do not use the same AppId value in installers for other applications.
; (To generate a new GUID, click Tools | Generate GUID inside the IDE.)
AppId={{8AB17DCE-0DB2-4E9C-BC3A-7C8434AED87F}
AppName={#__APP_NAME__}
AppVersion={#__APP_VERSION__}
;AppVerName={#__APP_NAME__} {#MyAppVersion}
AppPublisher={#__APP_PUBLISHER__}
AppPublisherURL={#__APP_URL__}
AppSupportURL={#__APP_URL__}
AppUpdatesURL={#__APP_URL__}
DefaultDirName={pf}\{#__APP_NAME__}
DefaultGroupName={#__APP_NAME__}
OutputDir=.
OutputBaseFilename=..\build\setup_rama
Compression=lzma
SolidCompression=yes

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked
Name: "quicklaunchicon"; Description: "{cm:CreateQuickLaunchIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked; OnlyBelowVersion: 0,6.1

[Files]
Source: "..\build\rama.exe"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\model_ALMA_coupler_Exy.lua"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\model_antenna_pattern_test.lua"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\model_horn.lua"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\model_schrodinger_test.lua"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\model_waveguide_Exy.lua"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\model_waveguide_Ez.lua"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\model_waveguide_bend.lua"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\doc\*.png"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\doc\*.gif"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\build\rama.html"; DestDir: "{app}"; Flags: ignoreversion
; NOTE: Don't use "Flags: ignoreversion" on any shared system files

[Icons]
Name: "{group}\{#__APP_NAME__}"; Filename: "{app}\{#__APP_EXE_NAME__}"
Name: "{group}\{cm:UninstallProgram,{#__APP_NAME__}}"; Filename: "{uninstallexe}"
Name: "{commondesktop}\{#__APP_NAME__}"; Filename: "{app}\{#__APP_EXE_NAME__}"; Tasks: desktopicon
Name: "{userappdata}\Microsoft\Internet Explorer\Quick Launch\{#__APP_NAME__}"; Filename: "{app}\{#__APP_EXE_NAME__}"; Tasks: quicklaunchicon

[Run]
Filename: "{app}\{#__APP_EXE_NAME__}"; Description: "{cm:LaunchProgram,{#StringChange(__APP_NAME__, '&', '&&')}}"; Flags: nowait postinstall skipifsilent

