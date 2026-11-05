@echo off
rem ---------------------------------------------------------------
rem  wt901.cmd - run the toolkit from any directory
rem
rem  Usage (from the project folder):
rem      .\wt901.cmd --version
rem      .\wt901.cmd scan
rem      .\wt901.cmd probe --address AA:BB:CC:DD:EE:FF
rem      .\wt901.cmd gui --sim
rem
rem  It sets PYTHONPATH to <this folder>\src, so the src layout works
rem  without installing the package and regardless of the current dir.
rem ---------------------------------------------------------------
setlocal
set "ROOT=%~dp0"
set "PYTHONPATH=%ROOT%src;%PYTHONPATH%"

where python >nul 2>nul
if errorlevel 1 (
    echo [ERROR] python not found in PATH.
    echo         Install Python 3.10+ and enable "Add python.exe to PATH".
    exit /b 1
)

python -m wt901 %*
exit /b %ERRORLEVEL%
