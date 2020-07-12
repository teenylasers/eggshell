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
DefaultDirName={autopf}\{#__APP_NAME__}
DefaultGroupName={#__APP_NAME__}
OutputDir=../build
OutputBaseFilename=setup_rama
Compression=lzma
SolidCompression=yes
ArchitecturesInstallIn64BitMode=x64

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked
Name: "quicklaunchicon"; Description: "{cm:CreateQuickLaunchIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked; OnlyBelowVersion: 0,6.1

[Files]
Source: "..\build\rama.exe"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\examples\*"; DestDir: "{app}\examples"; Flags: ignoreversion
Source: "..\build\images\*"; DestDir: "{app}\images"; Flags: ignoreversion
Source: "..\build\rama.html"; DestDir: "{app}"; Flags: ignoreversion
; NOTE: Don't use "Flags: ignoreversion" on any shared system files

[Icons]
Name: "{group}\{#__APP_NAME__}"; Filename: "{app}\{#__APP_EXE_NAME__}"
Name: "{group}\{cm:UninstallProgram,{#__APP_NAME__}}"; Filename: "{uninstallexe}"
Name: "{commondesktop}\{#__APP_NAME__}"; Filename: "{app}\{#__APP_EXE_NAME__}"; Tasks: desktopicon
Name: "{userappdata}\Microsoft\Internet Explorer\Quick Launch\{#__APP_NAME__}"; Filename: "{app}\{#__APP_EXE_NAME__}"; Tasks: quicklaunchicon

[Run]
Filename: "{app}\{#__APP_EXE_NAME__}"; Description: "{cm:LaunchProgram,{#StringChange(__APP_NAME__, '&', '&&')}}"; Flags: nowait postinstall skipifsilent

