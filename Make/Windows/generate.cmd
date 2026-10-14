@echo off
pushd "%~dp0"
bash ./generate
if exist RoboEireann.lnk goto linkExists
powershell "$s=(New-Object -COM WScript.Shell).CreateShortcut('%CD%\RoboEireann.lnk');$s.TargetPath='%CD%\..\..\Build\Windows\CMake\RoboEireann.sln';$s.Save()"
:linkExists
popd
