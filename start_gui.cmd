@echo off
rem ---------------------------------------------------------------
rem  start_gui.cmd - double-click launcher for the needle-angle GUI
rem
rem  No arguments : simulated sensor (no hardware needed)
rem  With args    : passed straight through, e.g.
rem      start_gui.cmd --sim
rem      start_gui.cmd --address AA:BB:CC:DD:EE:FF
rem      start_gui.cmd --name WT901
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

if "%~1"=="" (
    python -m wt901.gui --sim
) else (
    python -m wt901.gui %*
)

if errorlevel 1 (
    echo.
    echo [ERROR] GUI exited with code %ERRORLEVEL%.
    pause
)
exit /b %ERRORLEVEL%
