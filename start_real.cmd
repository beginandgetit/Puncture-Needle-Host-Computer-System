@echo off
rem ---------------------------------------------------------------
rem  start_real.cmd - launch the GUI against the paired IMU
rem
rem  Default device name: WT901BLE67 (rename it here if you change it).
rem  Extra arguments are passed through, e.g.
rem      start_real.cmd --address EA:D7:22:40:D3:0A
rem ---------------------------------------------------------------
setlocal
set "ROOT=%~dp0"
set "PYTHONPATH=%ROOT%src;%PYTHONPATH%"

where python >nul 2>nul
if errorlevel 1 (
    echo [ERROR] python not found in PATH.
    pause
    exit /b 1
)

python -m wt901.gui --name WT901BLE67 %*

if errorlevel 1 (
    echo.
    echo [ERROR] GUI exited with code %ERRORLEVEL%.
    pause
)
exit /b %ERRORLEVEL%
